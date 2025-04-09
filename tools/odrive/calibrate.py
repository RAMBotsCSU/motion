from time import sleep

import odrive
from odrive.enums import AxisState, MotorError, EncoderError

from fibre.libfibre import ObjectLostError

def connect():
    while True:
        try:
            dev = odrive.find_any(timeout=1)
            print("Connected.")
            sleep(1)
            return dev

        except TimeoutError:
            print("Failed to connect to odrive, waiting")
            sleep(1)


odr = connect()

try:
    odr.erase_configuration()
except ObjectLostError:
    pass

odr = connect()

odr.config.gpio7_mode = 0 #Set pin to digital
odr.config.gpio8_mode = 0 #Set pin to digital

odr.axis0.encoder.config.abs_spi_cs_gpio_pin = 7
odr.axis1.encoder.config.abs_spi_cs_gpio_pin = 8

for i, axis in enumerate([odr.axis0, odr.axis1]):
    print(f"Setting up axis{i}")

    axis.motor.config.pole_pairs = 20
    axis.encoder.config.mode = 257 #Set encoder to SPI mode
    axis.encoder.config.cpr = 16384

try:
    odr.save_configuration()
except ObjectLostError:
    pass

odr = connect()

odr.clear_errors()

for i, axis in enumerate([odr.axis0, odr.axis1]):
    print(f"Calibrating axis{i}")

    sleep(1)

    axis.requested_state = AxisState.MOTOR_CALIBRATION
    sleep(1)
    while(axis.current_state != 1):
        sleep(2)
        print(f"motor calibrating (err: {MotorError(axis.motor.error).name})")

    axis.motor.config.pre_calibrated = True

    axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
    sleep(1)
    while(axis.current_state != 1):
        sleep(2)
        print(f"encoder calibrating (err: {EncoderError(axis.encoder.error).name})")

    axis.encoder.config.pre_calibrated = True

    sleep(1)


odr.clear_errors()

try:
    odr.save_configuration()
except ObjectLostError:
    pass

print("Done!")
