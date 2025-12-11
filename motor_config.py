import odrive
from odrive.enums import *
import time

odrv0 = odrive.find_any() #to find odrive
print(str(odrv0.vbus_voltage)) #

print("Erasing pre-exsisting configuration...")
try:
    odrv0.erase_configuration()
except Exception:
    pass

odrv0 = odrive.find_any()

#odrive config
odrv0.config.enable_brake_resistor = True
odrv0.config.brake_resistance = 2.0  #(based on the resistor used for the Odrive)

#motor configuration
odrv0.axis1.motor.config.current_lim = 45 #check and compare with the website
odrv0.axis1.controller.config.vel_limit = 5 #minimum is 2 turn/s that is sloow
odrv0.axis1.motor.config.calibration_current = 7 #less than 15A
odrv0.config.dc_max_negative_current = -5 #as we have 24V power supply and brake resistor
odrv0.axis1.motor.config.pole_pairs = 7 #num of poles (14) / 2
odrv0.axis1.motor.config.torque_constant = 8.27/400 #formula 8.27/(num of KV)
odrv0.axis1.motor.config.motor_type = 0 

#encoder configuration
odrv0.axis1.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis1.encoder.config.cpr = 2000 #4* Pulse per revolution(500) for AS5047D encoder

#controller config
odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.config.pos_gain = 20.0
odrv0.axis1.controller.config.vel_gain = 0.16
odrv0.axis1.controller.config.vel_integrator_gain = 0.32
#first vel_gain tuned
#then pos gain tuned
#vel_integrator_gain = 0.5 * 10 * <vel_gain> is calculated


try:
    odrv0.save_configuration()
except:
    print('Config pass!')

#start calibration
odrv0.axis1.requested_state = 4 # MOTOR_CALIBRATION = 4 (0x4)
time.sleep(15)
odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True
print('Full Calibration Sequence for motor axis1')