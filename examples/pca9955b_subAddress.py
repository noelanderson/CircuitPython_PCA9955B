# SPDX-FileCopyrightText: Copyright (c) 2025 Noel Anderson
#
# SPDX-License-Identifier: Unlicense

import board
import busio

from pca9955b import PCA9955, SubAddress

# Create I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)


ledDriver = PCA9955(i2c, address=0x3F, oe_pin=board.GP10, reset_pin=board.GP11)
ledDriver.reset()
ledDriver.output_enable = True


# Print out current sub adddress values
# Defaults are
# SUBADR1: 0x76,      enabled: True
# SUBADR2: 0x76,      enabled: False
# SUBADR3: 0x76,      enabled: False
# ALLCALLADR: 0x70,   enabled: True

print(f"SUBADR1: {ledDriver.subaddresses[SubAddress.SUBADR1].address:#x},\
      enabled: {ledDriver.subaddresses[SubAddress.SUBADR1].enable}")

print(f"SUBADR2: {ledDriver.subaddresses[SubAddress.SUBADR2].address:#x},\
      enabled: {ledDriver.subaddresses[SubAddress.SUBADR2].enable}")

print(f"SUBADR3: {ledDriver.subaddresses[SubAddress.SUBADR3].address:#x},\
      enabled: {ledDriver.subaddresses[SubAddress.SUBADR3].enable}")

print(f"ALLCALLADR: {ledDriver.subaddresses[SubAddress.ALLCALLADR].address:#x},\
      enabled: {ledDriver.subaddresses[SubAddress.ALLCALLADR].enable}")

# Set Sub Address 3 and enable it
ledDriver.subaddresses[SubAddress.SUBADR3].address = 0x77
ledDriver.subaddresses[SubAddress.SUBADR3].enable = True

# verify the change
print(f"SUBADR3: {ledDriver.subaddresses[SubAddress.SUBADR3].address:#x},\
      enabled: {ledDriver.subaddresses[SubAddress.SUBADR3].enable}")
