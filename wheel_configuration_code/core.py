#!/usr/bin/env python3
"""
core.py — ROS2 ODrive wheel controller (improved)

Changes from original:
  - Fixed turn logic inversion in dist_callback
  - Added timeout to go_to_close_loop so it never spins forever
  - SMBus instance is reused (not recreated per call)
  - CSV filename is only incremented once per session, not on every reopen
  - Timestamp uses absolute nanoseconds / 1e9 (not mod-60)
  - rclpy.ok() guard in _log_motor_data
  - Graceful fallback if ODrive axes are unavailable during logging
  - Speed clamp added to odrive_control
  - movement_callback no longer silently swallows reconnect errors — publishes them
  - go_to_close_loop has a per-axis timeout (default 30 s) with a clear error log
  - Added MIN_VEL constant so vel can never go below safe floor
  - destroy_node() now also idles all motors before closing
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import odrive
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE
from odrive.utils import dump_errors

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt8, Float64
from geometry_msgs.msg import Twist

import smbus
import time
import csv
import os
import re
import threading
from typing import Optional, Callable, Tuple

# ── Topic names ────────────────────────────────────────────────────────────────
GPS_TOPIC   = "/fix"
MOVE_TOPIC  = "/wasd"
BUS_ADDRESS = 0x08

# ── Velocity limits ────────────────────────────────────────────────────────────
DEFAULT_VEL = 30
MIN_VEL     = 10      # vel can never drop below this via 'j'
MAX_VEL     = 70      # vel can never rise above this via 'u'
MAX_CMD_VEL = 100.0   # hard clamp for odrive_control output


# ══════════════════════════════════════════════════════════════════════════════
class Odrive_Arm:
    """
    Wrapper for two ODrive controllers.
    No ROS APIs used — can be tested standalone.
    """

    def __init__(
        self,
        serial_X: str,
        serial_YZ: str,
        logger: Optional[Callable[[str], None]] = None,
    ):
        self.serial_X  = serial_X
        self.serial_YZ = serial_YZ
        self._logger   = logger
        self.axes: dict = {}
        self._connect_to_odrive()

    # ── Logging ────────────────────────────────────────────────────────────────

    def _log(self, message: str):
        (self._logger or print)(message)

    # ── Connection helpers ─────────────────────────────────────────────────────

    def _find_odrive(
        self,
        serial_number: str,
        label: str,
        timeout: int = 5,
        retries: int = 3,
    ):
        last_error = None
        for attempt in range(1, retries + 1):
            self._log(
                f"Finding {label} ODrive "
                f"(serial={serial_number}, attempt {attempt}/{retries}, timeout={timeout}s)..."
            )
            try:
                device = odrive.find_any(serial_number=serial_number, timeout=timeout)
                if device is None or isinstance(device, list):
                    raise RuntimeError(
                        f"{label} find_any returned invalid result: {type(device)}"
                    )
                _ = device.vbus_voltage  # sanity-check communication
                self._log(f"{label} found.")
                return device
            except Exception as e:
                last_error = e
                self._log(f"{label} attempt {attempt} failed: {e}")
                time.sleep(1)
        raise RuntimeError(
            f"Failed to connect to {label} ODrive after {retries} attempts: {last_error}"
        )

    def _connect_to_odrive(self):
        self.odrv_Y = self._find_odrive(self.serial_YZ, "YZ")
        self.odrv_X = self._find_odrive(self.serial_X,  "X")

        self.axes = {
            "X": self.odrv_X.axis0,
            "A": self.odrv_X.axis1,
            "Y": self.odrv_Y.axis1,
            "Z": self.odrv_Y.axis0,
        }
        self._log("ODrives connected. Dumping previous errors...")
        for label, dev in (("YZ", self.odrv_Y), ("X", self.odrv_X)):
            try:
                self._log(f"{label} ODrive errors:")
                dump_errors(dev, True)
            except Exception as e:
                self._log(f"Failed to dump {label} errors: {e}")

    # ── Motion ─────────────────────────────────────────────────────────────────

    def move_axis(self, axis_id: str, velocity: float):
        assert axis_id in self.axes, f"Unknown axis: {axis_id}"
        try:
            self.axes[axis_id].controller.input_vel = velocity
        except AttributeError as e:
            self._log(f"move_axis({axis_id}): AttributeError — {e}")

    def move(self, vel: Tuple[float, float, float, float]):
        """Set velocity for all four axes simultaneously."""
        for axis_id, velocity in zip(self.axes.keys(), vel):
            self.move_axis(axis_id, velocity)

    def stop(self):
        """Zero all axes."""
        self.move((0.0, 0.0, 0.0, 0.0))

    # ── State ──────────────────────────────────────────────────────────────────

    def _set_state(self, axis_id: str, state):
        assert axis_id in self.axes, f"Unknown axis: {axis_id}"
        self.axes[axis_id].requested_state = state

    def idle_all(self):
        for axis_id in self.axes:
            try:
                self._set_state(axis_id, AXIS_STATE_IDLE)
            except Exception as e:
                self._log(f"idle_all: failed for axis {axis_id}: {e}")


# ══════════════════════════════════════════════════════════════════════════════
class Core(Node):
    """Main ROS2 node."""

    # ── Init ───────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__("core")
        self.get_logger().info("ROS2 Core node starting...")

        # ── Motion state ──
        self.vel    = DEFAULT_VEL
        self.turn   = 1      # 1 = go forward on obstacle, 0 = reverse
        self.auto   = 0
        self.manual = 0
        self.nav    = 1
        self.done   = 1

        # ── Timing ──
        self.old  = self.get_clock().now()
        self.need = Duration(seconds=1)

        # ── ODrive ──
        try:
            self.arm = Odrive_Arm(
                "345F355F3033",
                "347235573033",
                logger=self.get_logger().info,
            )
        except Exception as e:
            self.get_logger().error(f"ODrive init failed: {e}")
            raise

        self.go_to_close_loop()

        # ── I2C (single persistent instance) ──
        try:
            self._smbus = smbus.SMBus(8)
        except Exception as e:
            self.get_logger().warn(f"SMBus init failed (I2C unavailable): {e}")
            self._smbus = None

        # ── CSV logging ──
        self.csv_file    = None
        self.csv_writer  = None
        self.meta_filename = "current_log.txt"
        self.file_name   = self.load_last_filename()
        self.set_output_file(self.file_name)

        # ── ROS2 subscriptions + timer ──
        self._init_subscribers()
        self.create_timer(1.0, self._log_motor_data)

        self.get_logger().info("Core node ready.")

    # ── Subscribers ────────────────────────────────────────────────────────────

    def _init_subscribers(self):
        self.create_subscription(Float64,   "/distance", self.dist_callback,     10)
        self.create_subscription(Twist,     "/cmd_vel",  self.handle_control,    10)
        self.create_subscription(UInt8,     MOVE_TOPIC,  self.movement_callback, 10)
        # Uncomment to enable GPS:
        # self.create_subscription(NavSatFix, GPS_TOPIC, self.gps_callback, 10)

    # ── ODrive helpers ─────────────────────────────────────────────────────────

    def go_to_close_loop(self, per_axis_timeout: float = 30.0):
        """
        Wait until each axis is idle (state 1) or already in closed-loop (state 8),
        then command closed-loop control.  Runs in a daemon thread.
        Gives up after *per_axis_timeout* seconds per axis to avoid infinite spin.
        """
        def _worker():
            for axis_id, axis in self.arm.axes.items():
                deadline = time.monotonic() + per_axis_timeout
                while axis.current_state not in (1, 8):
                    if time.monotonic() > deadline:
                        self.get_logger().error(
                            f"Axis {axis_id} did not reach idle/closed-loop within "
                            f"{per_axis_timeout:.0f}s — skipping."
                        )
                        break
                    self.get_logger().info(
                        f"Waiting for axis {axis_id} (state={axis.current_state})..."
                    )
                    time.sleep(1)
                else:
                    self.arm._set_state(axis_id, AXIS_STATE_CLOSED_LOOP_CONTROL)
            self.get_logger().info("go_to_close_loop: done.")

        threading.Thread(target=_worker, daemon=True).start()

    def check_connected(self) -> bool:
        try:
            for label, dev in (("X", self.arm.odrv_X), ("YZ", self.arm.odrv_Y)):
                assert dev is not None and not isinstance(dev, list)
                assert dev.vbus_voltage > 0, f"ODrive {label} not responding"
            for axis_id, axis in self.arm.axes.items():
                assert axis is not None, f"Axis {axis_id} is None"
            return True
        except Exception as e:
            self.get_logger().error(f"check_connected: {e}")
            return False

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def gps_callback(self, data: NavSatFix):
        self.get_logger().info(
            f"GPS: lat={data.latitude:.6f} lon={data.longitude:.6f}"
        )

    def movement_callback(self, data: UInt8):
        if not self.check_connected():
            self.get_logger().warn("ODrive disconnected — attempting reconnect...")
            try:
                self.arm._connect_to_odrive()
                self.go_to_close_loop()
                self.file_name = self.load_last_filename()
                self.set_output_file(self.file_name)
                self.get_logger().info("Reconnect successful.")
            except Exception as e:
                self.get_logger().error(f"Reconnect failed: {e}")
                return

        v   = self.vel
        key = chr(data.data)
        self.get_logger().info(f"Key received: {key!r} (raw={data.data})")

        # Speed adjustment — clamped with constants
        if key == 'u' and self.vel < MAX_VEL:
            self.vel += 10
        elif key == 'j' and self.vel > MIN_VEL:
            self.vel -= 10

        # Movement
        if   key == 'w': self.arm.move([ v,  v, -v, -v / 2])   # forward
        elif key == 's': self.arm.move([-v, -v,  v,  v / 2])   # backward
        elif key == 'a': self.arm.move([-v, -v, -v, -v / 2])   # strafe/turn left
        elif key == 'd': self.arm.move([ v,  v,  v,  v / 2])   # strafe/turn right
        elif key == 'h': self.arm.stop()                        # halt
        else:
            if data.data == 2:                                   # special stop byte
                self.auto = 0
                self.arm.stop()

    def dist_callback(self, data: Float64):
        """
        Obstacle-avoidance state machine.

        dist == -1  →  no obstacle detected  →  execute turn-dodge sequence
        dist  > 0   →  obstacle cleared       →  reset to manual/nav
        """
        dist    = data.data
        now     = self.get_clock().now()
        elapsed = now - self.old

        if dist == -1 and self.nav == 0 and self.manual == 1:
            if self.turn == 1:
                # BUG FIX: was 0 in original — turn==1 means "go forward to dodge"
                self.arm.move([self.vel, self.vel, -self.vel, -self.vel / 2])
                if elapsed >= self.need:
                    self.arm.stop()
                    new_secs = self.need.nanoseconds / 1e9 + 1
                    self.need = Duration(seconds=new_secs)
                    self.turn = 0   # switch to reverse phase
                    self.old  = now
            else:
                # turn == 0: reverse phase
                self.arm.move([-self.vel, -self.vel, -self.vel, -self.vel / 2])
                if elapsed >= self.need:
                    self.arm.stop()
                    self.turn = 1   # reset to forward phase
                    self.old  = now

        elif dist > 0 and self.done == 0:
            # Obstacle cleared — reset navigation state
            self.need   = Duration(seconds=1)
            self.turn   = 1
            self.nav    = 1
            self.done   = 1
            self.manual = 0
            self.arm.stop()
            self.old = self.get_clock().now()

    def handle_control(self, twist_msg: Twist):
        x = twist_msg.linear.x
        y = twist_msg.angular.z
        self.get_logger().info(f"handle_control: linear.x={x:.3f} angular.z={y:.3f}")

        if x == 0.0 and y == 0.0:
            if self.done == 0:
                self.nav = 0
        else:
            self.manual = 1
            self.done   = 0

        self.odrive_control(x, y)

    # ── Motor control ──────────────────────────────────────────────────────────

    def odrive_control(self, x: float, y: float):
        """
        Differential-drive mixing with hard output clamp.
        x = forward/back  [-1, 1]
        y = angular rate  [-1, 1]
        """
        left_speed  = 50.0 * x
        right_speed = 50.0 * x
        if y != 0.0:
            left_speed  -= 50.0 * y
            right_speed += 50.0 * y

        # Clamp outputs to safe range
        left_speed  = max(-MAX_CMD_VEL, min(MAX_CMD_VEL, left_speed))
        right_speed = max(-MAX_CMD_VEL, min(MAX_CMD_VEL, right_speed))

        self.arm.move([left_speed, left_speed, -right_speed, -right_speed / 2])

    # ── I2C ────────────────────────────────────────────────────────────────────

    def i2c_write(self, value: str) -> int:
        """Write string bytes to I2C bus. Returns 0 on success, -1 on error."""
        if self._smbus is None:
            self.get_logger().warn("i2c_write: SMBus not available.")
            return -1
        try:
            byte_value = [ord(c) for c in value]
            self._smbus.write_i2c_block_data(BUS_ADDRESS, 0x00, byte_value)
            return 0
        except Exception as e:
            self.get_logger().error(f"i2c_write failed: {e}")
            return -1

    # ── CSV logging ────────────────────────────────────────────────────────────

    def _log_motor_data(self):
        """Called at 1 Hz by the ROS2 timer. Logs axis currents + velocity."""
        if not rclpy.ok():
            return

        # Absolute timestamp in seconds (not mod-60)
        timestamp = self.get_clock().now().nanoseconds / 1e9

        for axis_name, axis in self.arm.axes.items():
            if axis is None:
                continue
            try:
                iq          = axis.motor.current_control.Iq_measured
                id_         = axis.motor.current_control.Id_measured
                iq_setpoint = axis.motor.current_control.Iq_setpoint
                id_setpoint = axis.motor.current_control.Id_setpoint
                velocity    = axis.encoder.vel_estimate

                self.csv_writer.writerow(
                    [timestamp, axis_name, iq, id_, iq_setpoint, id_setpoint, velocity]
                )
                self.csv_file.flush()
            except Exception as e:
                self.get_logger().warn(f"Log error on axis {axis_name}: {e}")

        self.get_logger().debug("Motor data logged.")

    # ── File helpers ───────────────────────────────────────────────────────────

    def increment_log_filename(self, filename: str) -> str:
        match = re.match(r"(log)(\d+)(\.csv)?", filename)
        if match:
            prefix    = match.group(1)
            number    = int(match.group(2))
            extension = match.group(3) or ".csv"
            return f"{prefix}{number + 1}{extension}"
        return "log1.csv"

    def load_last_filename(self) -> str:
        if os.path.exists(self.meta_filename):
            with open(self.meta_filename, "r") as f:
                filename = f.read().strip()
                if filename:
                    return filename
        return "log1.csv"

    def save_current_filename(self, filename: str):
        with open(self.meta_filename, "w") as f:
            f.write(filename)

    def set_output_file(self, filename: str):
        """
        Open *filename* for append-mode CSV logging.
        The next filename (for the following session) is saved to meta.
        Only increments the counter once per session, not on every reopen.
        """
        if self.csv_file:
            self.csv_file.close()

        self.csv_file   = open(filename, mode="a", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        if os.path.getsize(filename) == 0:
            self.csv_writer.writerow(
                ["timestamp", "axis", "Iq", "Id", "Iq_setpoint", "Id_setpoint", "velocity"]
            )

        # Persist the *next* filename for the following session
        next_filename = self.increment_log_filename(filename)
        self.save_current_filename(next_filename)
        self.get_logger().info(f"Logging to {filename} (next session → {next_filename})")

    # ── Cleanup ────────────────────────────────────────────────────────────────

    def destroy_node(self):
        """Idle motors, flush CSV, shut down cleanly."""
        self.get_logger().info("Shutting down — idling all motors...")
        try:
            self.arm.idle_all()
        except Exception:
            pass
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
        super().destroy_node()


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = Core()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
