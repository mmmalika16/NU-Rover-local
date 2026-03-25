#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import odrive
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE
from odrive.utils import dump_errors

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt8, Float64   # NOTE: UInt8 replaces Char from ROS1
from geometry_msgs.msg import Twist

from typing import Tuple
import smbus
import time
import json
import csv
import os
import re
import threading
from typing import Optional, Callable

GPS_TOPIC   = "/fix"
MOVE_TOPIC  = "/wasd"
BUS_ADDRESS = 0x08


class Odrive_Arm:
    """
    Wrapper for two ODrive controllers.
    No ROS APIs used — identical to the original ROS1 version.
    """

    def __init__(self, serial_X: str, serial_YZ: str, logger: Optional[Callable[[str], None]] = None):
        self.serial_X  = serial_X
        self.serial_YZ = serial_YZ
        self._logger = logger
        self.axes: dict = {}
        self._connect_to_odrive()

    def _log(self, message: str):
        if self._logger is not None:
            self._logger(message)
        else:
            print(message, flush=True)

    def _find_odrive(self, serial_number: str, label: str, timeout: int = 5, retries: int = 3):
        last_error = None
        for attempt in range(1, retries + 1):
            self._log(
                f"Finding {label} ODrive (serial={serial_number}, attempt {attempt}/{retries}, timeout={timeout}s)..."
            )
            try:
                device = odrive.find_any(serial_number=serial_number, timeout=timeout)
                if device is None or isinstance(device, list):
                    raise RuntimeError(f"{label} find_any returned invalid result: {type(device)}")
                _ = device.vbus_voltage
                self._log(f"{label} is found")
                return device
            except Exception as e:
                last_error = e
                self._log(f"{label} connection attempt {attempt} failed: {e}")
                time.sleep(1)

        raise RuntimeError(f"Failed to connect to {label} ODrive after {retries} attempts: {last_error}")

    def _connect_to_odrive(self):
        self.odrv_Y = self._find_odrive(self.serial_YZ, "YZ")
        self.odrv_X = self._find_odrive(self.serial_X, "X")

        self.axes = {
            "X": self.odrv_X.axis0,
            "A": self.odrv_X.axis1,
            "Y": self.odrv_Y.axis1,
            "Z": self.odrv_Y.axis0,
        }
        self._log("ODrives connected. Dumping previous errors...")
        self._log("YZ ODrive Errors:")
        try:
            dump_errors(self.odrv_Y, True)
        except Exception as e:
            self._log(f"Failed to dump YZ errors: {e}")
        self._log("X ODrive Errors:")
        try:
            dump_errors(self.odrv_X, True)
        except Exception as e:
            self._log(f"Failed to dump X errors: {e}")

    def move_axis(self, axis_id: str, velocity: float):
        assert axis_id in self.axes
        try:
            self.axes[axis_id].controller.input_vel = velocity
        except AttributeError:
            pass

    def move(self, vel: Tuple[float, float, float, float]):
        for axis_id, velocity in zip(self.axes.keys(), vel):
            self.move_axis(axis_id, velocity)

    def _set_state(self, axis_id: str, state):
        assert axis_id in self.axes
        self.axes[axis_id].requested_state = state

    def _hold(self):
        for axis_id in self.axes:
            self._set_state(axis_id, AXIS_STATE_IDLE)


class Core(Node):
    """
    Main ROS2 node. Inherits from rclpy.node.Node.
    Replaces all rospy.* calls with rclpy equivalents.
    """

    def __init__(self):
        super().__init__("core")
        self.get_logger().info("ROS2 node initialized.")

        # ODrive interface
        self.turn  = 0
        self.auto  = 0
        self.vel   = 30
        try:
            self.arm = Odrive_Arm(
                "347235573033",
                "345F355F3033",
                logger=self.get_logger().info,
            )
        except Exception as e:
            self.get_logger().error(f"ODrive initialization failed: {e}")
            raise

        # go_to_close_loop runs in a background thread so it doesn't block callbacks
        self.ready = False
        self.go_to_close_loop()

        self.auto = 0

        # CSV logging
        self.csv_file    = None
        self.csv_writer  = None
        self.meta_filename = "current_log.txt"
        self.file_name   = self.load_last_filename()
        self.set_output_file(self.file_name)

        self.manual = 0
        self.nav    = 1
        self.done   = 1

        # ROS2: use get_clock().now() instead of rospy.Time.now()
        self.old  = self.get_clock().now()
        self.turn = 1

        # ROS2: use rclpy.duration.Duration instead of rospy.Duration
        self.need = Duration(seconds=1)

        self._init_subscribers()

        # ROS2: replace the while-loop + rate.sleep() pattern with a timer
        self.create_timer(1.0, self._log_motor_data)

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _init_subscribers(self):
        # ROS2: create_subscription(MsgType, topic, callback, qos_depth)
        self.create_subscription(Float64, "/distance",  self.dist_callback,     10)
        self.create_subscription(Twist,   "/cmd_vel",   self.handle_control,    10)
        self.create_subscription(UInt8,   MOVE_TOPIC,   self.movement_callback, 10)
        # Uncomment to enable GPS:
        # self.create_subscription(NavSatFix, GPS_TOPIC, self.gps_callback, 10)

    # ------------------------------------------------------------------
    # ODrive helpers
    # ------------------------------------------------------------------

    def go_to_close_loop(self):
        """
        Wait until each axis reaches idle/closed-loop state, then command
        closed-loop control. Runs in a daemon thread to avoid blocking the
        ROS2 executor.
        """
        def _worker():
            for axis_id in self.arm.axes:
                while self.arm.axes[axis_id].current_state not in (1, 8):
                    time.sleep(1)
                    self.get_logger().info(
                        f"Waiting for axis {axis_id} (state="
                        f"{self.arm.axes[axis_id].current_state})..."
                    )
                self.arm._set_state(axis_id, AXIS_STATE_CLOSED_LOOP_CONTROL)
            self.get_logger().info("All axes in closed-loop control.")
            self.ready = True

        threading.Thread(target=_worker, daemon=True).start()

    def check_connected(self) -> bool:
        try:
            assert self.arm.odrv_X is not None and not isinstance(self.arm.odrv_X, list)
            assert self.arm.odrv_Y is not None and not isinstance(self.arm.odrv_Y, list)
            assert self.arm.odrv_X.vbus_voltage > 0, "ODrive X not responding"
            assert self.arm.odrv_Y.vbus_voltage > 0, "ODrive YZ not responding"
            for axis_id, axis in self.arm.axes.items():
                assert axis is not None, f"Axis {axis_id} unavailable"
            return True
        except Exception as e:
            self.get_logger().error(f"ODrive connection check failed: {e}")
            return False

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def gps_callback(self, data: NavSatFix):
        gps_data = {"latitude": data.latitude, "longitude": data.longitude}
        self.get_logger().info(f"GPS: {gps_data}")

    def movement_callback(self, data: UInt8):
        if not self.ready:
            self.get_logger().warn("ODirves are not ready yet.")
            return

        # Reconnect if ODrive was lost
        if not self.check_connected():
            try:
                self.arm._connect_to_odrive()
                self.go_to_close_loop()
                self.file_name = self.load_last_filename()
                self.set_output_file(self.file_name)
            except Exception as e:
                self.get_logger().error(f"Reconnect failed: {e}")
                return

        v = self.vel
        key = chr(data.data)
        self.get_logger().info(f"Key received: {key!r} (raw={data.data})")

        # Speed adjustment
        if   key == 'u' and self.vel < 70: self.vel += 10
        elif key == 'j' and self.vel > 30: self.vel -= 10

        # Movement
        if   key == 'w': self.arm.move([ v,  v, -v, -v / 2])
        elif key == 'a': self.arm.move([-v, -v, -v, -v / 2])
        elif key == 's': self.arm.move([-v, -v,  v,  v / 2])
        elif key == 'd': self.arm.move([ v,  v,  v,  v / 2])
        elif key == 'h': self.arm.move([0, 0, 0, 0])
        else:
            if data.data == 2:
                self.auto = 0
                self.arm.move([0, 0, 0, 0])

    def dist_callback(self, data: Float64):
        dist    = data.data
        now     = self.get_clock().now()
        elapsed = now - self.old  # returns a Duration in ROS2

        if dist == -1 and self.nav == 0 and self.manual == 1:
            if self.turn == 0:
                self.arm.move([self.vel, self.vel, -self.vel, -self.vel / 2])
                # ROS2: compare Duration objects, not plain ints
                if elapsed >= self.need:
                    self.arm.move([0, 0, 0, 0])
                    new_secs = self.need.nanoseconds / 1e9 + 1
                    self.need = Duration(seconds=new_secs)
                    self.turn = 0
                    self.old  = now
            else:
                self.arm.move([-self.vel, -self.vel, -self.vel, -self.vel / 2])
                if elapsed >= self.need:
                    self.arm.move([0, 0, 0, 0])
                    self.turn = 1
                    self.old  = now

        elif dist > 0 and self.done == 0:
            self.need   = Duration(seconds=1)
            self.turn   = 1
            self.nav    = 1
            self.done   = 1
            self.manual = 0
            self.arm.move([0, 0, 0, 0])
            self.old = self.get_clock().now()

    def handle_control(self, twist_msg: Twist):
        x = twist_msg.linear.x
        y = twist_msg.angular.z
        self.get_logger().info(f"handle_control: x={x:.3f} y={y:.3f}")

        if x == 0 and y == 0:
            if self.done == 0:
                self.nav = 0
        else:
            self.manual = 1
            self.done   = 0

        self.odrive_control(x, y)

    # ------------------------------------------------------------------
    # Motor control
    # ------------------------------------------------------------------

    def odrive_control(self, x: float, y: float):
        left_speed  = 50 * x
        right_speed = 50 * x
        if y != 0:
            left_speed  -= 50 * y
            right_speed += 50 * y
        self.arm.move([left_speed, left_speed, -right_speed, -right_speed / 2])

    # ------------------------------------------------------------------
    # I2C
    # ------------------------------------------------------------------

    def i2c_write(self, value: str) -> int:
        byte_value = [ord(c) for c in value]
        smbus.SMBus(8).write_i2c_block_data(BUS_ADDRESS, 0x00, byte_value)
        return -1

    # ------------------------------------------------------------------
    # CSV logging — called by 1 Hz timer (replaces while loop)
    # ------------------------------------------------------------------

    def _log_motor_data(self):
        timestamp = self.get_clock().now().nanoseconds / 1e9 % 60
        for axis_name, axis in self.arm.axes.items():
            if axis is None:
                continue
            try:
                iq         = axis.motor.current_control.Iq_measured
                id_        = axis.motor.current_control.Id_measured
                iq_setpoint = axis.motor.current_control.Iq_setpoint
                id_setpoint = axis.motor.current_control.Id_setpoint
                velocity   = axis.encoder.vel_estimate
                self.csv_writer.writerow(
                    [timestamp, axis_name, iq, id_, iq_setpoint, id_setpoint, velocity]
                )
                self.csv_file.flush()
            except Exception as e:
                self.get_logger().warn(f"Error logging {axis_name}: {e}")
        self.get_logger().info("Motor data logged.")

    # ------------------------------------------------------------------
    # File helpers (identical to ROS1 version)
    # ------------------------------------------------------------------

    def increment_log_filename(self, filename: str) -> str:
        match = re.match(r"(log)(\d+)(\.csv)?", filename)
        if match:
            prefix    = match.group(1)
            number    = int(match.group(2))
            extension = match.group(3) if match.group(3) else ".csv"
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
        if self.csv_file:
            self.csv_file.close()
        self.csv_file   = open(filename, mode="a", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        if os.path.getsize(filename) == 0:
            self.csv_writer.writerow(
                ["timestamp", "axis", "Iq", "Id", "Iq_setpoint", "Id_setpoint", "velocity"]
            )
        self.file_name = self.increment_log_filename(filename)
        self.save_current_filename(self.file_name)
        self.get_logger().info(f"Logging to {filename}")

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        """Ensure CSV is flushed and closed on shutdown."""
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = Core()
    try:
        rclpy.spin(node)        # replaces rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()