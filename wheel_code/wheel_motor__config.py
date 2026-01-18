import odrive
from odrive.enums import *
import time

odrv0 = odrive.find_any() #to find odrive
print(str(odrv0.vbus_voltage))

print("Erasing pre-exsisting configuration...")
try:
    odrv0.erase_configuration()
except Exception:
    pass

print("Wait unitl reboot")
time.sleep(5)

odrv0 = odrive.find_any()
time.sleep(1)

#odrive config
odrv0.config.enable_brake_resistor = True
odrv0.config.brake_resistance = 2.0  #(based on the resistor used for the Odrive)

#motor configuration
odrv0.axis1.motor.config.current_lim = 45 #check and compare with the website
odrv0.axis1.controller.config.vel_limit = 5 #minimum is 2 turn/s that is sloow
odrv0.axis1.motor.config.calibration_current = 10 #less than 15A
odrv0.config.dc_max_negative_current = -5 #as we have 24V power supply and brake resistor
odrv0.axis1.motor.config.pole_pairs = 7 #num of poles (14) / 2
odrv0.axis1.motor.config.torque_constant = 8.27/400 #formula 8.27/(num of KV)
odrv0.axis1.motor.config.motor_type = 0 

#encoder configuration
odrv0.axis1.encoder.config.mode = 0 #ENCODER_MODE_INCREMENTAL
odrv0.axis1.encoder.config.cpr = 2000 #4* Pulse per revolution(500) for AS5047D encoder

#controller config
odrv0.axis1.controller.config.control_mode = 2 #CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.config.pos_gain = 10.0
odrv0.axis1.controller.config.vel_gain = 0.02
odrv0.axis1.controller.config.vel_integrator_gain = 5*odrv0.axis1.controller.config.vel_gain


print("Save configurations")
try:
    odrv0.save_configuration()
except:
    print('Config pass!')

time.sleep(5)
odrv0 = odrive.find_any()
time.sleep(1)

#start calibration
odrv0.axis1.requested_state = 3 # MOTOR_CALIBRATION = 4 (0x4) && AXIS_STATE_FULL_CALIBRATIO_SEQUENCE = 3
time.sleep(15)
odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True
odrv0.axis1.config.startup_encoder_offset_calibration = True
print('Full Calibration Sequence for motor axis1')

try:
    odrv0.save_configuration()
except:
    print('Config pass!')