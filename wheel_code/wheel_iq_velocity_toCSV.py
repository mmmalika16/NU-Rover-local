import odrive
import time
import csv
from odrive.enums import (
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL
)

# Open velocity CSV file
velocity_csv_file = open("wheels_motor_velocity_data.csv", "w", newline="")
velocity_writer = csv.writer(velocity_csv_file, delimiter="\t")
velocity_writer.writerow(["time", "vel_setpoint", "vel_estimate"])

# Open current CSV file
current_csv_file = open("wheels_motor_current_data.csv", "w", newline="")
current_writer = csv.writer(current_csv_file, delimiter="\t")
current_writer.writerow(["time", "iq_setpoint", "iq_measured"])

# Connect to ODrive
odrv0 = odrive.find_any()
print("Connected!")

# Calibrate if not calibrated
if not odrv0.axis1.motor.config.pre_calibrated:
    print("Motor is not calibrated")
    odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    while odrv0.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(1)

    odrv0.axis1.motor.config.pre_calibrated = True
    odrv0.axis1.encoder.config.pre_calibrated = True
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(1)

# Velocity control
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
start = time.time()

# Logging function
def log_velocity():
    t = format(time.time() - start, ".6f").replace(".", ",")
    vel_s = format(odrv0.axis1.controller.input_vel, ".6f").replace(".", ",")
    vel_e = format(odrv0.axis1.encoder.vel_estimate, ".6f").replace(".", ",")
    velocity_writer.writerow([t, vel_s, vel_e])
    velocity_csv_file.flush()

def log_current():
    t = format(time.time() - start, ".6f").replace(".", ",")
    iq_m = format(odrv0.axis1.motor.current_control.Iq_measured, ".6f").replace(".", ",")
    iq_s = format(odrv0.axis1.motor.current_control.Iq_setpoint, ".6f" ).replace(".", ",") 
    current_writer.writerow([t, iq_s, iq_m])
    current_csv_file.flush()

# Run test
print("Input vel = 30")
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.controller.input_vel = 30

for _ in range(300):
    log_velocity()
    log_current()
    time.sleep(0.01)

odrv0.axis1.controller.input_vel = 0
odrv0.axis1.requested_state = AXIS_STATE_IDLE

velocity_csv_file.close()
current_csv_file.close()
print("Finished. Saved to wheels_motor_data.csv")
print("pos_gain = ", odrv0.axis1.controller.config.pos_gain)
print("vel_gain = ", odrv0.axis1.controller.config.vel_gain)
print("vel_integrator_gain = ", odrv0.axis1.controller.config.vel_integrator_gain)