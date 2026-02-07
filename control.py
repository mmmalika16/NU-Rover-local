#!/usr/bin/env python3

import odrive
import os
import time
import csv
from pathlib import Path
from typing import Dict, Optional, List, Tuple
from odrive.utils import dump_errors
from odrive.enums import (
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_POSITION_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL
)
import serial
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import ikpy.chain
import sys
import math

# Add custom message path
two_up = os.path.abspath(os.path.join(__file__, "..", "..", ".."))
sys.path.append(two_up)

# from main.msg import JoyCmd
from sensor_msgs.msg import Joy

# Configuration Constants
URDF_PATH = "/home/jetsonnano/roboarm_control_ros2_ws/src/main/main/ARM.urdf"
ARDUINO_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Mega_2560_96505111011523193640-if00"
ARDUINO_BAUDRATE = 115200 #communication speed
CSV_LOG_PATH = "/home/jetson/logs/motor_currents6.csv"

# Safety limits for each joint (degrees)
SAFETY_LIMITS = {
    "base": {"min": -230, "max": 270},
    "shoulder": {"min": -2, "max": 180},
    "elbow": {"min": -2, "max": 270}
}

# Gear ratios for each joint
GEAR_RATIOS = {
    "base": 120,
    "shoulder": 120,
    "elbow": 100,
    "wrist1": 100,
    "wrist2": 100
}

#calculates the inverse kinematics for the desired position, given the current joint angles as a starting point.
def move(chain, x: float = 0, y: float = 0, z: float = 0,
         base: float = 0, shoulder: float = 0, elbow: float = 0) -> Optional[np.ndarray]:
    """
    Calculate inverse kinematics for desired position
    
    Args:
        chain: IKPy kinematic chain
        x, y, z: Cartesian position offset
        base, shoulder, elbow: Current joint angles in degrees
        
    Returns:
        Array of target angles [base, shoulder, elbow] in degrees, or None if invalid
    """
    try:
        PREV_ANGLES = np.radians([0, base, shoulder, elbow, 0]) #IK works with radians
        previous_position = chain.forward_kinematics([0, 0, 0, 0, 0]) #finds current end effector position based on current joint angles
        previous_position[:3, 3] += [x, y, z] #apply the desired offset to the current position to get the target position
        target_angles = chain.inverse_kinematics_frame(
            target=previous_position,
            initial_position=PREV_ANGLES
        ) #calculate the joint angles needed to reach the target position, starting from the current angles as an initial guess
        return np.round(np.degrees(target_angles), 4)[1:4] #converts values to degrees
    except Exception as e:
        print(f"IK calculation error: {e}")
        return None

#main ROS node
class ArmController(Node):
    """ROS 2 Node for controlling ODrive-based robotic arm"""
    
    def __init__(self, serial_first: str, serial_second: str, serial_third: str):
        super().__init__("arm_controller") #give a name arm_controller to the node
        self.get_logger().info("Starting Arm Controller (ROS 2 Humble)")

        # State variables
        self.current_position = [0.0, 0.0, 0.0]
        self.prev_position = None
        self.connected = False
        self.rebooted = False
        
        # ODrive serial numbers
        self.serial_first = serial_first
        self.serial_second = serial_second
        self.serial_third = serial_third
        
        # ODrive instances
        self.odrv_first = None
        self.odrv_second = None
        self.odrv_third = None
        self.axes: Dict = {}
        
        # Gear ratios
        self.gear_ratio = GEAR_RATIOS
        
        # Arduino connection
        self.arduino_serial = None
        self.find_arduino()
        
        # CSV logging
        self.csv_file = None
        self.csv_writer = None
        self.setup_csv_logging()
        
        # ROS 2 timers and rates
        self.last_joy_time = self.get_clock().now() #limits joystick update rate
        self.joy_interval = Duration(seconds=0.1)  # 100ms interval of joystick processing.
        
        # Kinematic chain
        if os.path.exists(URDF_PATH):
            self.chain = ikpy.chain.Chain.from_urdf_file(
                URDF_PATH,
                active_links_mask=[False, True, True, True, False]
            )
        else:
            self.get_logger().error(f"URDF file not found: {URDF_PATH}")
            self.chain = None
        
        # Connect to ODrives
        self._connect_to_odrives()
        
        # ROS 2 subscriptions
        self.joy_subscription = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )
        
        # Timers for periodic tasks
        self.error_check_timer = self.create_timer(1.0, self.check_motor_errors) #every second checks motor errors
        self.data_log_timer = self.create_timer(0.1, self.log_motor_data)  # 10Hz logging

    def setup_csv_logging(self):
        """Initialize CSV logging for motor data"""
        try:
            log_dir = Path(CSV_LOG_PATH).parent
            log_dir.mkdir(parents=True, exist_ok=True)
            
            self.csv_file = open(CSV_LOG_PATH, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "time", "axis_name", "Iq_measured", "Id_measured",
                "Iq_setpoint", "Id_setpoint", "pos_estimate", "pos_setpoint",
                "vel_estimate", "vel_setpoint"
            ])
            self.get_logger().info(f"CSV logging initialized: {CSV_LOG_PATH}")
        except Exception as e:
            self.get_logger().error(f"Failed to setup CSV logging: {e}")

    def find_arduino(self):
        """Attempt to connect to Arduino"""
        self.get_logger().info(f"Waiting for Arduino on {ARDUINO_PORT}...")
        
        max_attempts = 1
        attempt = 0
        
        while rclpy.ok() and attempt < max_attempts:
            if os.path.exists(ARDUINO_PORT):
                try:
                    self.arduino_serial = serial.Serial(
                        port=ARDUINO_PORT,
                        baudrate=ARDUINO_BAUDRATE,
                        timeout=1.0
                    )
                    time.sleep(0.1)  # Allow serial to stabilize
                    self.arduino_serial.write(b'S\n')
                    self.get_logger().info(f"Successfully connected to Arduino on {ARDUINO_PORT}")
                    return
                except serial.SerialException as e:
                    self.get_logger().warn(f"Failed to connect to Arduino: {e}")
            else:
                self.get_logger().warn(f"{ARDUINO_PORT} not found. Retrying...")
            
            attempt += 1
            time.sleep(1)
        
        self.get_logger().warn("Arduino connection failed after max attempts. Continuing without Arduino.")

    def _connect_to_odrives(self):
        """Connect to all three ODrives and initialize their axes"""
        self.get_logger().info("Connecting to ODrives...")
        
        try:
            # Find first ODrive
            self.get_logger().info(f"Finding first ODrive (SN: {self.serial_first})...")
            self.odrv_first = odrive.find_any(serial_number=self.serial_first)
            if not self.odrv_first:
                raise RuntimeError(f"ODrive with serial {self.serial_first} not found!")
            self.get_logger().info("First ODrive connected")

            # Find second ODrive
            self.get_logger().info(f"Finding second ODrive (SN: {self.serial_second})...")
            self.odrv_second = odrive.find_any(serial_number=self.serial_second)
            if not self.odrv_second:
                raise RuntimeError(f"ODrive with serial {self.serial_second} not found!")
            self.get_logger().info("Second ODrive connected")

            # Find third ODrive
            self.get_logger().info(f"Finding third ODrive (SN: {self.serial_third})...")
            self.odrv_third = odrive.find_any(serial_number=self.serial_third)
            if not self.odrv_third:
                raise RuntimeError(f"ODrive with serial {self.serial_third} not found!")
            self.get_logger().info("Third ODrive connected")

            # Mapping ODrive axes
            self.axes = {
                "base": self.odrv_first.axis1,
                "shoulder": self.odrv_second.axis1,
                "elbow": self.odrv_second.axis0,
                "wrist1": self.odrv_third.axis1,
                "wrist2": self.odrv_third.axis0
            }

            # Setup each motor
            for axis_name, axis in self.axes.items():
                if axis is None:
                    self.get_logger().warn(f"Axis {axis_name} is None, skipping setup")
                    continue
                
                # Wait for axis to be idle or in closed loop
                timeout = 30
                start_time = time.time()
                axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                time.sleep(0.5) # Ensure it starts in idle
                while axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                    if time.time() - start_time > timeout:
                        self.get_logger().error(f"Timeout waiting for {axis_name} to be ready")
                        break
                    time.sleep(0.1)
                
                self.setup_motor(axis, axis_name)

            self.get_logger().info("ODrives connected. Dumping previous errors:")
            self.log_errors()

            self.connected = True
            self.get_logger().info("All ODrives successfully connected and configured")
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to ODrives: {e}")
            self.connected = False

    def setup_motor(self, axis, name: str):
        """Ensure the motor is calibrated and ready for movement"""
        try:
            if not axis.motor.config.pre_calibrated:
                self.get_logger().info(f"Calibrating motor on {name}...")
                axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                
                # Wait for calibration with timeout
                timeout = 30
                start_time = time.time()
                while axis.current_state != AXIS_STATE_IDLE:
                    if time.time() - start_time > timeout:
                        self.get_logger().error(f"Calibration timeout for {name}")
                        return
                    time.sleep(0.5)

                axis.motor.config.pre_calibrated = True
                self.get_logger().info(f"Motor on {name} calibrated")
            
            # Enter closed loop control
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.get_logger().info(f"Motor on {name} is ready")
            
        except Exception as e:
            self.get_logger().error(f"Failed to setup motor {name}: {e}")

    def stop_motors(self):
        """Stop all motors safely"""
        self.get_logger().info("Stopping all motors...")
        for axis_name, axis in self.axes.items():
            if axis is None:
                continue
            try:
                axis.requested_state = AXIS_STATE_IDLE
                time.sleep(0.1)
                axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            except Exception as e:
                self.get_logger().error(f"Error stopping {axis_name}: {e}")
        self.get_logger().info("All motors stopped")

    def reboot(self):
        """Reboot all ODrives and reset initial positions"""
        self.connected = False
        self.get_logger().info("Rebooting ODrives...")
        
        odrives = [
            ("first", self.odrv_first),
            ("second", self.odrv_second),
            ("third", self.odrv_third)
        ]
        
        for name, odrv in odrives:
            if odrv:
                try:
                    odrv.reboot()
                    self.get_logger().info(f"{name.capitalize()} ODrive rebooted")
                except Exception as e:
                    self.get_logger().warn(f"Error rebooting {name} ODrive: {e}")
        
        time.sleep(3)  # Wait for reboot
        self.get_logger().info("ODrives rebooted")

    def log_errors(self):
        """Dump errors from all ODrives"""
        self.get_logger().info("Dumping ODrive errors...")
        
        odrives = [
            ("First", self.odrv_first),
            ("Second", self.odrv_second),
            ("Third", self.odrv_third)
        ]
        
        for name, odrv in odrives:
            if odrv:
                try:
                    dump_errors(odrv)
                except Exception as e:
                    self.get_logger().warn(f"Error dumping errors from {name} ODrive: {e}")
            else:
                self.get_logger().warn(f"{name} ODrive is not connected or uninitialized")

    def check_motor_errors(self):
        """Periodic check for motor errors"""
        if not self.connected:
            return
        
        for axis_name, axis in self.axes.items():
            if axis is None:
                continue

            try:
                axis_error = getattr(axis, "error", 0)
                motor_error = getattr(axis.motor, "error", 0) if hasattr(axis, "motor") else 0
                encoder_error = getattr(axis.encoder, "error", 0) if hasattr(axis, "encoder") else 0

                if axis_error != 0 or motor_error != 0 or encoder_error != 0:
                    self.get_logger().error(
                        f"Error detected on {axis_name}: "
                        f"axis={axis_error}, motor={motor_error}, encoder={encoder_error}"
                    )
                    self.log_errors()
                    # Optionally trigger reboot
                    # self.reboot()
                    # self._connect_to_odrives()
                    
            except AttributeError as e:
                self.get_logger().warn(f"Could not read error state from {axis_name}: {e}")

    def check_connected(self) -> bool:
        """Verify ODrive connections are still active"""
        try:
            assert self.odrv_first is not None
            assert self.odrv_second is not None
            assert self.odrv_third is not None
            # Check if ODrives respond
            assert self.odrv_first.vbus_voltage > 0, "ODrive 1 not responding"
            assert self.odrv_second.vbus_voltage > 0, "ODrive 2 not responding"
            assert self.odrv_third.vbus_voltage > 0, "ODrive 3 not responding"

            # Ensure axes are available
            for axis_id, axis in self.axes.items():
                assert axis is not None, f"Axis {axis_id} is not available"

            return True

        except Exception as e:
            self.get_logger().warn(f"ODrive connection check failed: {e}")
            return False

    def move_arm(self, position: List[float]):
        """Move arm to cartesian position using inverse kinematics"""
        if self.chain is None:
            self.get_logger().error("Kinematic chain not initialized")
            return
        
        x, y, z = position
        
        # Get current joint angles
        last_angle = {
            "base": 0.0,
            "shoulder": 0.0,
            "elbow": 0.0,
            "wrist1": 0.0,
            "wrist2": 0.0
        }
        
        for i, (axis_name, axis) in enumerate(self.axes.items()):
            if axis is None and i >= 3:
                continue
            
            try:
                if self.rebooted:
                    last_angle[axis_name] = (self.current_position[i] / self.gear_ratio[axis_name]) * 360
                else:
                    last_angle[axis_name] = (axis.encoder.pos_estimate / self.gear_ratio[axis_name]) * 360
            except Exception as e:
                self.get_logger().warn(f"Error reading position from {axis_name}: {e}")

        # Calculate inverse kinematics
        angles = move(self.chain, x, y, z,
                     last_angle["base"], last_angle["shoulder"], last_angle["elbow"])
        
        if angles is None:
            self.get_logger().warn("Invalid IK solution, movement aborted")
            return

        self.move_to_angle(angles)

    def move_to_angle(self, angles: np.ndarray):
        """Move joints to specified angles with safety checks"""
        validation_results = self.validate_angles(angles, SAFETY_LIMITS)

        if not any(validation_results):
            self.get_logger().warn(f"All motions exceed safe limits! Movement aborted. Angles: {angles}")
            return

        motor_turns = {}
        axes_names = ["base", "shoulder", "elbow"]
        
        for i, (axis_name, angle) in enumerate(zip(axes_names, angles)):
            if not validation_results[i]:
                self.get_logger().warn(
                    f"{axis_name} angle {angle}° out of bounds "
                    f"[{SAFETY_LIMITS[axis_name]['min']}, {SAFETY_LIMITS[axis_name]['max']}]. Skipping."
                )
                # Reset to previous position
                if self.prev_position is not None and i < len(self.prev_position):
                    self.current_position[i] = self.prev_position[i]
                else:
                    self.current_position[i] = 0.0
                continue

            # Special case: prevent base movement when shoulder is too low
            if axis_name == "base" and angles[1] < 70:
                self.get_logger().warn(f"Shoulder angle {angles[1]}° too low for base movement. Skipping base.")
                continue

            # Calculate motor turns and send command
            motor_turns[axis_name] = (angle / 360.0) * self.gear_ratio[axis_name]
            
            try:
                self.axes[axis_name].controller.input_pos = motor_turns[axis_name]
            except Exception as e:
                self.get_logger().error(f"Error setting position for {axis_name}: {e}")

        self.get_logger().info(
            f"Moving to angles: {[f'{angles[i]:.2f}' if validation_results[i] else 'SKIP' for i in range(3)]}° "
            f"(motor turns: {motor_turns})"
        )

    def move_wrist_velocity(self, wrist_key: str, velocity: float):
        """Set wrist velocity"""
        if wrist_key not in self.axes or self.axes[wrist_key] is None:
            self.get_logger().warn(f"Wrist axis {wrist_key} not available")
            return
        
        try:
            # Set to velocity control mode if needed
            if self.axes[wrist_key].controller.config.control_mode != CONTROL_MODE_VELOCITY_CONTROL:
                self.axes[wrist_key].controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            
            self.axes[wrist_key].controller.input_vel = velocity
            self.get_logger().debug(f"Setting {wrist_key} velocity to {velocity}")
        except Exception as e:
            self.get_logger().error(f"Error setting velocity for {wrist_key}: {e}")

    def validate_angles(self, angles: np.ndarray, limits: Dict) -> List[int]:
        """Validate that angles are within safe limits"""
        axes_names = ["base", "shoulder", "elbow"]
        outputs = []
        
        for i, angle in enumerate(angles):
            if i >= len(axes_names):
                break
            
            axis_name = axes_names[i]
            if limits[axis_name]["min"] <= angle <= limits[axis_name]["max"]:
                outputs.append(1)
            else:
                outputs.append(0)

        return outputs

    def write_to_arduino(self, data: bytes):
        """Write data to Arduino with error handling"""
        if self.arduino_serial is None:
            return
        
        try:
            if self.arduino_serial.is_open:
                self.arduino_serial.write(data)
            else:
                raise serial.SerialException("Serial port not open")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")
            try:
                self.arduino_serial.close()
            except:
                pass
            self.find_arduino()  # Try to reconnect
        except Exception as e:
            self.get_logger().error(f"Unexpected error during serial write: {e}")

    def joy_callback(self, msg: Joy):
        """Handle joystick input commands"""
        if not self.connected:
            self.get_logger().warn("ODrives not connected, ignoring joystick input")
            return

        # Rate limiting
        now = self.get_clock().now()
        if now - self.last_joy_time < self.joy_interval:
            return
        self.last_joy_time = now

        # Handle button commands (Arduino control)
        if msg.buttons[0]:
            self.write_to_arduino(b'F\n')  # Forward
        elif msg.buttons[1]:
            self.write_to_arduino(b'B\n')  # Backward
        elif not msg.buttons[0] and not msg.buttons[1]:
            self.write_to_arduino(b'S\n')  # Stop
            

        # Check connection status
        if not self.check_connected():
            self.get_logger().warn("Connection lost, attempting reconnection...")
            self._connect_to_odrives()
            return

        # Handle wrist velocity controls
        velocity2 = msg.axes[5] * 3
        velocity1 = msg.axes[4] * 8
        
        self.move_wrist_velocity("wrist1", velocity1)
        self.move_wrist_velocity("wrist2", velocity2)

        # Handle position commands
        if msg.axes[0] == 0 and msg.axes[1] == 0 and msg.axes[2] == 0:
            return  # No movement command

        scale = 0.03
        dx = msg.axes[0] * scale * -1
        dy = msg.axes[1] * scale
        dz = msg.axes[3] * scale

        # Update position
        self.prev_position = self.current_position.copy()
        self.current_position[0] += dx
        self.current_position[1] += dy
        self.current_position[2] += dz

        self.get_logger().debug(
            f"dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} v1={velocity1:.2f} v2={velocity2:.2f}"
        )
        self.get_logger().debug(f"Position: {self.current_position}")
        
        self.move_arm(self.current_position)

    def log_motor_data(self):
        """Log motor data to CSV file"""
        if not self.connected or self.csv_writer is None:
            return
        
        timestamp = self.get_clock().now().nanoseconds / 1e9
        
        for axis_name, axis in self.axes.items():
            if axis is None:
                continue
            
            try:
                iq = axis.motor.current_control.Iq_measured
                id_measured = axis.motor.current_control.Id_measured
                iq_setpoint = axis.motor.current_control.Iq_setpoint
                id_setpoint = axis.motor.current_control.Id_setpoint
                pos_estimate = axis.encoder.pos_estimate
                pos_setpoint = axis.controller.pos_setpoint
                vel_estimate = axis.encoder.vel_estimate
                vel_setpoint = axis.controller.vel_setpoint
                
                # Log to CSV
                self.csv_writer.writerow([
                    timestamp, axis_name, iq, id_measured,
                    iq_setpoint, id_setpoint, pos_estimate, pos_setpoint,
                    vel_estimate, vel_setpoint
                ])
                self.csv_file.flush()
                
            except Exception as e:
                self.get_logger().warn(f"Error logging data from {axis_name}: {e}")

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down Arm Controller...")
        
        # Stop motors
        if self.connected:
            self.stop_motors()
        
        # Close CSV file
        if self.csv_file:
            self.csv_file.close()
        
        # Close Arduino connection
        if self.arduino_serial and self.arduino_serial.is_open:
            self.arduino_serial.close()
        
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    # Replace with actual ODrive serial numbers
    controller = ArmController(
        serial_first="316932743431",
        serial_second="316732753431",
        serial_third="316932493431"
    )
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Keyboard interrupt received")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()