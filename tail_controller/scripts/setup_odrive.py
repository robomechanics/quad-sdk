import odrive
from odrive.enums import *
from odrive.utils import *

print("finding an odrive...")
odrv0 = odrive.find_any()
print("found an odrive")

dump_errors(odrv0, True)
# odrv0.erase_configuration()

# Odrive can estimate the resistance by itself, the resistance below is inaccurate
# odrv0.config.brake_resistance = 5 # [Ohm]

# Reference: https://store-en.tmotor.com/goods.php?id=317

# Axis0 is pitch
odrv0.axis0.motor.config.current_lim = 60 # [A]
odrv0.axis0.controller.config.vel_limit = 25 # [turn/s]

# Odrive can estimate the pole_pairs by itself
# odrv0.axis0.motor.config.pole_pairs = 14 / 2
# Odrive can estimate the torque_constant by itself, the constant below is wrong
# odrv0.axis0.motor.config.torque_constant = 8.27 / 700 # [8.27 / (motor KV)]
# Odrive can estimate the motor_type by itself
# odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

odrv0.axis0.encoder.config.cpr = 4000
odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis0.encoder.config.use_index = True

odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv0.axis0.current_state != AXIS_STATE_IDLE:
     time.sleep(0.1)

odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.motor.config.pre_calibrated = True

# odrv0.axis0.config.startup_motor_calibration = True
odrv0.axis0.config.startup_encoder_index_search = True
# odrv0.axis0.config.startup_encoder_offset_calibration = True

# Axis1 is roll
odrv0.axis1.motor.config.current_lim = 60 # [A]
odrv0.axis1.controller.config.vel_limit = 25 # [turn/s]

# Odrive can estimate the pole_pairs by itself
# odrv0.axis1.motor.config.pole_pairs = 14 / 2
# Odrive can estimate the torque_constant by itself, the constant below is wrong
# odrv0.axis1.motor.config.torque_constant = 8.27 / 700 # [8.27 / (motor KV)]
# Odrive can estimate the motor_type by itself
# odrv0.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

odrv0.axis1.encoder.config.cpr = 4000
odrv0.axis1.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis1.encoder.config.use_index = True

odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv0.axis1.current_state != AXIS_STATE_IDLE:
     time.sleep(0.1)

odrv0.axis1.encoder.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True

# odrv0.axis1.config.startup_motor_calibration = True
odrv0.axis1.config.startup_encoder_index_search = True
# odrv0.axis1.config.startup_encoder_offset_calibration = True

odrv0.save_configuration()
odrv0.reboot()
