# SPDX-FileCopyrightText: Copyright (c) 2025 Noel Anderson
#
# SPDX-License-Identifier: Unlicense


# This sample uses the global features to continious blink channels 0 and 3.
# Once started this blinking pattern will run without additional program input, until stopped

import time

import board
import busio

from pca9955b import PCA9955, LedChannel

# Create I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)


ledDriver = PCA9955(i2c, address=0x3F, oe_pin=board.GP10, reset_pin=board.GP11)
ledDriver.reset()
ledDriver.output_enable = True


# Global settings
ledDriver.brightness = 0x7F  # 50% brightness
ledDriver.gain = 0xFF  # Max

# Use channels 0 & 3
ledDriver.channels[0].state = LedChannel.PWM_GRP
ledDriver.channels[3].state = LedChannel.PWM_GRP

# Blink leds approx once per second with a 50% duty cycle
ledDriver.pwm = 0x7F  # 50% duty cycle (pwm/256) as per data sheet 7.3.4
ledDriver.frequency = 15  # ~1 sec (frequency+1/15.6) as per data sheet 7.3.5

# Start
ledDriver.blinking = True

time.sleep(30)

# Stop
ledDriver.blinking = False
ledDriver.channels[0].state = LedChannel.OFF
