import odrive
from odrive.enums import CONTROL_MODE_VELOCITY_CONTROL, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_CLOSED_LOOP_CONTROL
import time

odrv0 = odrive.find_any()
print(str(odrv0.vbus_voltage))

#sensorless ramp params
odrv0.axis1.config.sensorless_ramp.current = 5.0
odrv0.axis1.config.sensorless_ramp.vel = 220.0
odrv0.axis1.config.sensorless_ramp.accel = 150.0

# motor config
odrv0.axis1.motor.config.current_lim = 2 * odrv0.axis1.config.sensorless_ramp.current
odrv0.axis1.motor.config.calibration_current = 15
odrv0.axis1.motor.config.pole_pairs = 7
odrv0.axis1.motor.config.torque_constant = 8.27/400
odrv0.axis1.motor.config.motor_type = 0

# sensorless controller
odrv0.axis1.controller.config.vel_gain = 0.01
odrv0.axis1.controller.config.vel_integrator_gain = 0.05
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.config.vel_limit = odrv0.axis1.config.sensorless_ramp.vel*1.4 / (2 * 3.14 * 7)
odrv0.axis1.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (7 * 400)
odrv0.axis1.config.enable_sensorless_mode = True

try:
    odrv0.save_configuration()
except:
    pass

odrv0 = odrive.find_any()
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
time.sleep(12) 

odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True
try:
    odrv0.save_configuration()
except:
    pass

print("Sensorless calibration done.")

# velocity control
odrv0 = odrive.find_any()
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(0.5)
odrv0.axis1.controller.input_vel = 1.0
time.sleep(5)
odrv0.axis1.controller.input_vel = 0.0 #stops
print("Done")