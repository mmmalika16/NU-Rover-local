#!/usr/bin/env python3

import odrive
import os   #we need that for file paths, check if file exists.
import time
import csv
from pathlib import Path
from typing import Dict, Optional, List, Tuple
from odrive.utils import dump_errors
from odrive.enums import (
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL
)
import serial # For Arduino communication via USB. 
import rclpy #ROS2
from rclpy.node import Node #allows creating Nodes in ROS2
from rclpy.duration import Duration #for handling time intervals in ROS2
# from rclpy.executors import MultiThreadedExecutor
import numpy as np
import ikpy.chain #inverse kinematics library
import sys
import math

# Add custom message path
two_up = os.path.abspath(os.path.join(__file__, "..", "..", ".."))
sys.path.append(two_up)

# from main.msg import JoyCmd
from sensor_msgs.msg import Joy

# Configuration Constants
URDF_PATH = "/home/jetsonnano/roboarm_control_ros2_ws/src/main/main/ARM.urdf"
CSV_LOG_PATH = "/home/jetsonnano/roboarm_control_ros2_ws/motor_currents6.csv"


class ArmController(Node):
    """ROS 2 Node for controlling ODrive-based robotic arm"""
    
    def __init__(self, serial_third: str):
        super().__init__("arm_controller")
        self.get_logger().info("Starting Arm Controller (ROS 2 Humble)")

        # State variables
        self.connected = False
        self.rebooted = False
        
        # ODrive serial numbers
        self.serial_third = serial_third
        
        # ODrive instances
        self.odrv_third = None
        self.axes: Dict = {}
                
        # CSV logging
        self.csv_file = None
        self.csv_writer = None
        self.setup_csv_logging()
        
        # ROS 2 timers and rates
        self.last_joy_time = self.get_clock().now() #limits joystick update rate
        self.joy_interval = Duration(seconds=0.1)  # 100ms interval
        
        # Kinematic chain
        '''THIS PART READ WHAT IT DOES AS WELL AS ASM.URDF FILE'''
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
        self.error_check_timer = self.create_timer(1.0, self.check_motor_errors)
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

    def _connect_to_odrives(self):
        """Connect to all three ODrives and initialize their axes"""
        self.get_logger().info("Connecting to ODrives...")
        
        try:
            
            # Find third ODrive
            self.get_logger().info(f"Finding third ODrive (SN: {self.serial_third})...")
            self.odrv_third = odrive.find_any(serial_number=self.serial_third)
            if not self.odrv_third:
                raise RuntimeError(f"ODrive with serial {self.serial_third} not found!")
            self.get_logger().info("Third ODrive connected")

            # Mapping ODrive axes
            self.axes = {
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
                time.sleep(0.5)
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
            ("third", self.odrv_third)
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
            assert self.odrv_third is not None
            # Check if ODrives respond
            assert self.odrv_third.vbus_voltage > 0, "ODrive 3 not responding"

            # Ensure axes are available
            for axis_id, axis in self.axes.items():
                assert axis is not None, f"Axis {axis_id} is not available"

            return True

        except Exception as e:
            self.get_logger().warn(f"ODrive connection check failed: {e}")
            return False

    def move_wrist_velocity(self, wrist_key: str, velocity: float):
        """Set wrist velocity"""
        if wrist_key not in self.axes or self.axes[wrist_key] is None:
            self.get_logger().warn(f"Wrist axis {wrist_key} not available")
            return
        
        try:
            self.axes[wrist_key].controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.axes[wrist_key].controller.input_vel = velocity
            self.get_logger().info(f"Setting {wrist_key} velocity to {velocity}")
            # self.get_logger().debug(f"Setting {wrist_key} velocity to {velocity}")
        except Exception as e:
            self.get_logger().error(f"Error setting velocity for {wrist_key}: {e}")
    

    
    def joy_callback(self, msg: Joy):
        """Handle joystick input commands"""
        if not self.connected:
            self.get_logger().info(f"Axes: {msg.axes}")
            self.get_logger().warn("ODrives not connected, ignoring joystick input")
            return

        # Rate limiting
        now = self.get_clock().now()
        if now - self.last_joy_time < self.joy_interval:
            return
        self.last_joy_time = now

        # Check connection status
        if not self.check_connected():
            self.get_logger().warn("Connection lost, attempting reconnection...")
            self._connect_to_odrives()
            return

        # Handle wrist velocity controls
        velocity2 = msg.axes[0] * 3
        velocity1 = msg.axes[1] * 5
        
        self.move_wrist_velocity("wrist1", velocity1)
        self.move_wrist_velocity("wrist2", velocity2)

        
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
        
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    # Replace with actual ODrive serial numbers
    controller = ArmController(
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