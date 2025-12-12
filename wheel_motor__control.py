import odrive
import time
import csv
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, CONTROL_MODE_VELOCITY_CONTROL

# Open CSV file
csv_file = open("wheels_motor_data.csv", "w", newline="")
writer = csv.writer(csv_file)
writer.writerow(["time", "Iq_measured", "Id_measured", "Iq_setpoint", "Id_setpoint", "vel_cmd","vel_estimate"])

#connect odrive
odrv0 = odrive.find_any()
print("Connected!")

#calibrate if not calibrated
if not odrv0.axis1.motor.config.pre_calibrated:
    print("Motor is not calibrated")
    odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while odrv0.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(1)
    odrv0.axis1.motor.config.pre_calibrated = True
    odrv0.axis1.encoder.config.pre_calibrated = True
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL #8
    time.sleep(1)


odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL#2
start = time.time()


#"vel_estimate", "vel_setpoint"
def log_row(vel_cmd):
    writer.writerow([
        time.time() - start,
        odrv0.axis1.motor.current_control.Iq_measured,
        odrv0.axis1.motor.current_control.Id_measured,
        odrv0.axis1.motor.current_control.Iq_setpoint,
        odrv0.axis1.motor.current_control.Id_setpoint,
        vel_cmd,
        odrv0.axis1.encoder.vel_estimate
    ])
    csv_file.flush()

#vel = float(input("Enter first velocity command (in counts/s): "))
print("Input vel = 1")
odrv0.axis0.controller.input_vel = 1
for _ in range(200):   # 200 * 0.01 sec = 2 seconds
    log_row(1)
    time.sleep(0.01)

csv_file.close()
print("Finished. Saved to log.csv")