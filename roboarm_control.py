#!/usr/bin/env python3
import odrive
import os #using stuff of operating system
import time
import csv
from odrive.utils import dump_errors
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_POS_FILTER, INPUT_MODE_TRAP_TRAJ, AXIS_STATE_HOMING
import serial #serial communication for Arduino, maybe we don`t need it.`
# import rospy - this is for the ROS1
import rclpy #for ros2
from rclpy.node import Node
from rclpy.duration import Duration
from fibre.libfibre import EmptyInterface # Odrive communication stuff

import numpy as np
import ikpy.chain #inverse kinematics library

import sys #interpreter
import math
import time

two_up = os.path.abspath(os.path.join(__file__, "..", "..", ".."))
sys.path.append(two_up) #check packages that are two levels up

from main.msg import JoyCmd #importing custom message type

URDF_PATH = "/home/jetson/catkin_ws/src/roboarm_control/src/ARM.urdf" #change the path

port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Mega_2560_96505111011523193640-if00"  # Arduino port
baudrate = 115200 # how fast the communication is
arduino_serial = None

csv_file = open("/home/jetson/logs/motor_currents6.csv", "w", newline="") #change the path
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["time", "axis_name", "Iq_measured", "Id_measured", "Iq_setpoint", "Id_setpoint", "pos_estimate", "pos_setpoint", "vel_estimate", "vel_setpoint"])

def move(chain, x=0, y=0, z=0, base=0, shoulder=0, elbow=0):
    PREV_ANGLES = np.radians([0, base, shoulder, elbow, 0])
    previous_position = chain.forward_kinematics([0, 0, 0, 0, 0])
    previous_position[:3, 3] += [x, y, z]
    target_angles = chain.inverse_kinematics_frame(target=previous_position, initial_position=PREV_ANGLES)
    return np.round(np.degrees(target_angles), 4)[1:4]

class ArmController(Node):
    def __init__(self, serial_first: str, serial_second: str, serial_third: str):
        super().__init__("arm_controller") # Initialize ROS 2 node
        self.get_logger().info("Starting Arm Controller (ROS 2)") #ROS 2 print

        self.current_position = [0, 0, 0]
        self.prev_position = None

        self.serial_first = serial_first
        self.serial_second = serial_second
        self.serial_third = serial_third

        self.find_arduino()

        self.gear_ratio = {
            "base": 120,
            "shoulder": 120,
            "elbow": 100,
            "wrist1": 100,
            "wrist2": 100
        }
        self.connected = False
        self.rebooted = False
        
        self.create_timer(1.0, self.timer_callback)
        self.error_check_rate = self.create_rate(1.0)  # 1 Hz error checking
        self.last_joy_time = self.get_clock().now()
        self.joy_interval = Duration(seconds=0.1) # 100ms interval
        self.axes = {}  # Dictionary to store ODrive axes
        self.chain = ikpy.chain.Chain.from_urdf_file(URDF_PATH, active_links_mask=[False, True, True, True, False])
        self._connect_to_odrives()
    
        self.create_subscription(
            JoyCmd,
            "/joy_cmd",
            self.joy_callback,
            10
        )


    #DO WE NEED AN ARDUOINO?
    def find_arduino(self):
        global arduino_serial
        self.get_logger().info(f"Waiting for Arduino on {port}...")
    
        while rclpy.ok():
            if os.path.exists(port):
                try:
                    arduino_serial = serial.Serial(port=port, baudrate=baudrate)
                    arduino_serial.write(b'S\n')
                    self.get_logger().info(f"Successfully connected to Arduino on {port}")
                    return
                except serial.SerialException as e:
                    self.get_logger().warn(f"Failed to connect to Arduino on {port}: {e}")
            else:
                self.get_logger().warn(f"{port} not found. Retrying in 1 second...")
            time.sleep(1)

    def _connect_to_odrives(self):
        # Connects to all three ODrives and initializes their axes.
        print("Finding first ODrive...")
        self.odrv_first = odrive.find_any(serial_number=self.serial_first)
        if not self.odrv_first:
           raise RuntimeError(f"ODrive with serial {self.serial_first} not found!")

        print("Finding second ODrive...")
        self.odrv_second = odrive.find_any(serial_number=self.serial_second)
        if not self.odrv_second or isinstance(self.odrv_second, EmptyInterface):
            self.get_logger().error("ODrive2 connection failed or device is uninitialized.")
            return

        print("Finding third ODrive...")
        self.odrv_third = odrive.find_any(serial_number=self.serial_third)
        if not self.odrv_third or isinstance(self.odrv_third, EmptyInterface):
            self.get_logger().error("ODrive3 connection failed or device is uninitialized.")
            return
        

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
            if axis == None:
                continue
            while axis.current_state != 1 and axis.current_state != 8:
                time.sleep(1)
            self.setup_motor(axis, axis_name)

        print("ODrives connected. Dumping previous errors:")
        self.log_errors()

        self.connected = True
        
    def setup_motor(self, axis, name):
        """Ensures the motor is calibrated and ready for movement."""
        if not axis.motor.config.pre_calibrated:
            print(f"Calibrating motor on {name}...")
            axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            while axis.current_state != AXIS_STATE_IDLE:
                time.sleep(1)  # Wait for calibration

            axis.motor.config.pre_calibrated = True
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        print(f"Motor on {name} is ready.")

    def stop_motors(self):
        """Stops all motors safely."""
        for axis in self.axes.values():
            axis.requested_state = AXIS_STATE_IDLE
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        print("All motors stopped.")
                
    def reboot(self):
        """Reboots all ODrives and resets the initial positions."""
        self.connected = False
        print("Rebooting ODrives...")
        try:
            self.odrv_first.reboot()
        except:
            print('First ODrive rebooted')
        try:
            self.odrv_second.reboot()
        except:
            print('Second ODrive rebooted')
        try:
            self.odrv_third.reboot()
        except:
            print('Third ODrive rebooted')
        print("ODrives rebooted.")

    def log_errors(self):
        print("Dumping errors...")
        if self.odrv_first and not isinstance(self.odrv_first, EmptyInterface):
            dump_errors(self.odrv_first)
        else:
            self.get_logger().warn("First ODrive is not connected or uninitialized.")

        if self.odrv_second and not isinstance(self.odrv_second, EmptyInterface):
            dump_errors(self.odrv_second)
        else:
            self.get_logger().warn("Second ODrive is not connected or uninitialized.")

        if self.odrv_third and not isinstance(self.odrv_third, EmptyInterface):
            dump_errors(self.odrv_third)
        else:
            self.get_logger().warn("Third ODrive is not connected or uninitialized.")

    def check_motor_errors(self):
        print("motor error checking")
        for axis in self.axes.values():
            if axis == None:
                continue

            try:
                axis_error = getattr(axis, "error", 0)
                motor_error = getattr(axis.motor, "error", 0) if hasattr(axis, "motor") else 0
                encoder_error = getattr(axis.encoder, "error", 0) if hasattr(axis, "encoder") else 0

                if axis_error != 0 or motor_error != 0 or encoder_error != 0:
                    self.log_errors()
                    self.reboot()
                    self.rebooted = True
                    self._connect_to_odrives()
            except AttributeError as e:
                self.get_logger().warn(f"[ODrive Warning] Could not read error state from axis {name}: {e}")

    def check_connected(self):
        try:
            assert self.odrv_second is not None and not isinstance(self.odrv_second, list) and not isinstance(self.odrv_second, EmptyInterface)
            assert self.odrv_third is not None and not isinstance(self.odrv_third, list) and not isinstance(self.odrv_third, EmptyInterface)

            # Check if ODrive responds
            assert self.odrv_second.vbus_voltage > 0, "ODrive X is not responding"
            assert self.odrv_third.vbus_voltage > 0, "ODrive YZ is not responding"

            # Ensure axes are available
            for axis_id, axis in self.axes.items():
                assert axis is not None, f"Axis {axis_id} is not available"

            return True  # ODrive is connected

        except Exception as e:
            print(f"ODrive connection check failed: {e}")
            return False  # ODrive is not connected


    def move_arm(self, position):
        x, y, z = position
        last_angle = {
            "base": 0,
            "shoulder": 0,
            "elbow": 0,
            "wrist1": 0,
            "wrist2": 0
        }
        for i, (axis_name, axis) in enumerate(self.axes.items()):
            if axis == None and i >= 3:
                continue
            if self.rebooted:
                last_angle[axis_name] = (self.current_position[i] / self.gear_ratio[axis_name]) * 360
            else:
                last_angle[axis_name] = (axis.encoder.pos_estimate / self.gear_ratio[axis_name]) * 360

        self.angles = move(self.chain, x, y, z, last_angle["base"], last_angle["shoulder"], last_angle["elbow"])
        if self.angles is None:
            print("Invalid movement detected.")
            return

        self.move_to_angle(self.angles)

    def move_to_angle(self, angles):
        """Moves the specified gearbox to the specified angle with safety checks."""
        # Define safe motion limits for each axis
        safety_limits = {
            "base": {"min": -230, "max": 270},  # Base rotation limits in degrees
            "shoulder": {"min": -2, "max": 180},    # Shoulder limits
            "elbow": {"min": -2, "max": 270}  # Elbow limits
        }

        validation_results = self.validate_angles(angles, safety_limits)

        if not any(validation_results):
            print("WARNING: All motions exceed safe limits! Movement aborted.")
            print(angles)
            return

        motor_turns = {}
        for i, (axis_name, angle) in enumerate(zip(["base", "shoulder", "elbow"], angles)):
            if validation_results[i] == 0:
                print(f"WARNING: {axis_name} angle {angle} degrees out of bounds. Skipping.")
                print(f"SKIPPED position: {self.current_position[i]} and previous position: {self.prev_position[i] if self.prev_position else 'None'}")
                self.current_position[i] = 0 if self.prev_position is None else self.prev_position[i]
                print(f"Resetting {axis_name} to {self.current_position[i]} degrees")
                continue

            if axis_name == "base" and angles[1] < 70:
                print(f"WARNING: shoulder angle {angle} degrees is too low for base movement. Skipping.")
                continue
           
            motor_turns[axis_name] = (angle / 360) * self.gear_ratio[axis_name]
            self.axes[axis_name].controller.input_pos = motor_turns[axis_name]

        print(f"Moving to {[angles[i] if validation_results[i] else 'SKIPPED' for i in range(3)]} degrees ({motor_turns} motor turns)")

    def move_wrist_velocity(self, wrist_key, velocity):
        print(f"Setting {wrist_key} velocity to {velocity}")
        
        self.axes[wrist_key].controller.input_vel = velocity


    def validate_angles(self, angles, limits):
        """Validate that angles are within safe limits"""
        axes_names = ["base", "shoulder", "elbow"]
        outputs = [1, 1, 1]
        for i, angle in enumerate(angles):
            if i >= len(axes_names):
                break
                
            axis_name = axes_names[i]
            if angle < limits[axis_name]["min"] or angle > limits[axis_name]["max"]:
                outputs[i] = 0

        return outputs

    def write_to_arduino(self, data):
        global arduino_serial
        try:
            if arduino_serial and arduino_serial.is_open:
                arduino_serial.write(data)
            else:
                raise serial.SerialException("Serial port not open")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")
            try:
                arduino_serial.close()
            except:
                pass
            self.find_arduino()  # Try to reconnect
        except Exception as e:
            self.get_logger().error(f"Unexpected error during serial write: {e}")

    def joy_callback(self, msg: JoyCmd):
        if self.connected == False:
            print("Odrives are not connected")
            return

        now = self.get_clock().now()
        if now - self.last_joy_time < self.joy_interval:
            return  # Skip this message
        self.last_joy_time = now

        if len(msg.buttons) >= 3:
            if msg.buttons[0]:
                self.write_to_arduino(b'F\n')  # Forward
            elif msg.buttons[1]:
                self.write_to_arduino(b'B\n')  # Backward
            elif not msg.buttons[0] and not msg.buttons[1]:
                self.write_to_arduino(b'S\n')  # Stop
            if msg.buttons[2]:
                print("reboot button is pressed")
                self.reboot()
                self.rebooted = False
                self.current_position = [0, 0, 0]
                self._connect_to_odrives()
        
        connected = self.check_connected()
        if connected == False:
            self._connect_to_odrives()

        velocity2 = msg.j4 * 2
        velocity1 = msg.j5 * 2
        
        self.move_wrist_velocity("wrist1", velocity1)
        self.move_wrist_velocity("wrist2", velocity2)

        if msg.x == 0 and msg.y == 0 and msg.z == 0:
            print("no dx dy dz")
            return

        scale = 0.03
        dx = msg.x * scale * -1  # X movement
        dy = msg.y * scale * -1  # Y movement
        dz = msg.z * scale * -1  # Z movement

        self.prev_position = self.current_position.copy()

        self.current_position[0] += dx
        self.current_position[1] += dy
        self.current_position[2] += dz

        print(f"dx = {dx} dy = {dy} dz = {dz} velocity1 = {velocity1} velocity2 = {velocity2}")
        print(f"current_position: {self.current_position}")
        self.move_arm(self.current_position)

    def spin(self):
        while rclpy.ok():
            if self.connected:
                timestamp = self.get_clock().now().nanoseconds / 1e9
                for axis_name, axis in self.axes.items():
                    if axis is None:
                        continue
                    try:
                        iq = axis.motor.current_control.Iq_measured
                        id = axis.motor.current_control.Id_measured
                        iq_setpoint = axis.motor.current_control.Iq_setpoint
                        id_setpoint = axis.motor.current_control.Id_setpoint
                        pos_estimate = axis.encoder.pos_estimate
                        pos_setpoint = axis.controller.pos_setpoint
                        vel_estimate = axis.encoder.vel_estimate
                        vel_setpoint = axis.controller.vel_setpoint
                        # Log to CSV
                        csv_writer.writerow([timestamp, axis_name, iq, id, iq_setpoint, id_setpoint, pos_estimate, pos_setpoint, vel_estimate, vel_setpoint])
                        csv_file.flush()
                    except Exception as e:
                        self.get_logger().warn(f"Error reading current from {axis_name}: {e}")
            
                self.check_motor_errors()
            self.error_check_rate.sleep()

if __name__ == "__main__":
    # Replace with actual ODrive serial numbers
    rclpy.init()
    controller = ArmController("316932743431", "316732753431", "316932493431")
    rclpy.spin(controller)
    # controller.destroy_node()
    # rclpy.shutdown()
