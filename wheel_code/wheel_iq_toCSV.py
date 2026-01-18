import odrive
import time
import csv
from odrive.enums import (
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL
)

# Open CSV file
csv_file = open("wheels_motor_data.csv", "w", newline="")
writer = csv.writer(csv_file, delimiter="\t")
writer.writerow(["time", "Iq_measured", "Iq_setpoint"])

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
def log_row():
    t = format(time.time() - start, ".6f").replace(".", ",")
    iq_m = format(odrv0.axis1.motor.current_control.Iq_measured, ".6f").replace(".", ",")
    iq_s = format(odrv0.axis1.motor.current_control.Iq_setpoint, ".6f" ).replace(".", ",") 
    writer.writerow([t, iq_m, iq_s])
    csv_file.flush()

# Run test
print("Input vel = 30")
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.controller.input_vel = 30

for _ in range(300):
    log_row()
    time.sleep(0.01)

odrv0.axis1.controller.input_vel = 0
odrv0.axis1.requested_state = AXIS_STATE_IDLE

csv_file.close()
print("Finished. Saved to wheels_motor_data.csv")