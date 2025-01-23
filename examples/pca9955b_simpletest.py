# SPDX-FileCopyrightText: Copyright (c) 2025 Noel Anderson
#
# SPDX-License-Identifier: Unlicense

import time

import board
import busio

from pca9955b import PCA9955, LedChannel

# Create I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)


ledDriver = PCA9955(i2c, address=0x3F, oe_pin=board.GP10, reset_pin=board.GP11)
ledDriver.reset()
ledDriver.output_enable = True


ledDriver.gain = 0xFF
ledDriver.brightness = 0x7F  # 50% brightness

# Manually turn on, then off the first channel
ledDriver.channels[0].state = LedChannel.ON
time.sleep(5)

ledDriver.channels[0].state = LedChannel.OFF
time.sleep(5)

# Gradualy increase brightness from off to fully bright for channel 12
ledDriver.channels[12].state = LedChannel.PWM
for i in range(255):
    ledDriver.channels[12].brightness = i
    time.sleep(0.02)
