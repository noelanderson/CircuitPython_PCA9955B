# SPDX-FileCopyrightText: Copyright (c) 2025 Noel Anderson
#
# SPDX-License-Identifier: Unlicense


# This sample uses the group feature to run channels 0 and 13 in a continious sawwave pattern.
# Once started the group will run without any additional input, until stopped

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

# Configure the channels for graduation & assign to group 0
ledDriver.channels[0].output_state = LedChannel.PWM_GRP
ledDriver.channels[0].groupId = 0
ledDriver.channels[0].graduation_mode = True

ledDriver.channels[13].output_state = LedChannel.PWM_GRP
ledDriver.channels[13].groupId = 0
ledDriver.channels[13].graduation_mode = True

# Configure the group.
# Creates a group with two channels automatically following this sawtooth pattern -
#  * gradually ramp on
#  * hold on for 0.5 second
#  * turn off
#  * hold off for 0.5 seconds
# Repeats till stopped
#    _   _   _   _   _   _   _
#   / |_/ |_/ |_/ |_/ |_/ |_/ |
#
ledDriver.groups[0].factor_per_step = 32
ledDriver.groups[0].cycle_time = 1  # 0.5ms

ledDriver.groups[0].ramp_rate = 10
ledDriver.groups[0].ramp_up = True
ledDriver.groups[0].ramp_down = False

ledDriver.groups[0].hold_off = True
ledDriver.groups[0].hold_off_time = 1  # 0.5s

ledDriver.groups[0].hold_on = True
ledDriver.groups[0].hold_on_time = 1  # 0.5s

ledDriver.groups[0].gain = 0x7F


# Start the group running. This will run the pattern continuously without
# any CPU intervention until manually stopped
ledDriver.groups[0].graduation_mode = 1  # Continuous
ledDriver.groups[0].graduation_start()

time.sleep(30)

# Stop the group
ledDriver.groups[0].graduation_stop()
