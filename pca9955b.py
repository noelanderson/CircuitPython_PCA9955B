# SPDX-FileCopyrightText: Copyright (c) 2024 Noel Anderson
#
# SPDX-License-Identifier: MIT
"""
`pca9955b`
================================================================================

CircuitPython helper library for the NXP 16-Channel I²C-Bus Constant-Current LED Driver


* Author(s): Noel Anderson

Implementation Notes
--------------------

**Hardware:**

* `PCA9955 <https://www.nxp.com/docs/en/data-sheet/PCA9955B.pdf>`

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads


"""

# imports

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/NoelAnderson/CircuitPython_PCA9955B.git"

from micropython import const

try:
    from typing import Optional, NoReturn
    from busio import I2C
except ImportError:
    pass

import adafruit_bus_device.i2c_device as i2c_device

from digitalio import DigitalInOut, Direction, DriveMode
import microcontroller
import time

_PCA9955B_DEFAULT_I2C_ADDR = const(0x3F) # AD10 AD1 & AD2 all FLT (Floating Inputs)

# Register map & bit positions

_PCA9955_REG_MODE1 = const(0x00)   # R/W
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#  AIF  |  AI1  |  AI0  | SLEEP |  SUB1 |  SUB2 |  SUB3 |ALLCALL|
#---------------------------------------------------------------#
_PCA9955_BIT_ALLCALL = const(0)  #R
_PCA9955_BIT_SUB3 = const(1)  # R/W
_PCA9955_BIT_SUB2 = const(2)  # R/W
_PCA9955_BIT_SUB1 = const(3)  # R/W
_PCA9955_BIT_SLEEP = const(4)  # R/W
_PCA9955_BIT_AI0 = const(5)  # R/W
_PCA9955_BIT_AI1 = const(6)  # R/W
_PCA9955_BIT_AIF = const(7)  # R/W

_PCA9955_REG_MODE2 = const(0x01)   # R/W
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
# OVERTP| ERROR | DMBLNK| CLRERR|  OCH  |EXP_EN |   -   |   -   |
#---------------------------------------------------------------#
_PCA9955_BIT_EXP_EN = const(2)  # R/W
_PCA9955_BIT_OCH = const(3)  # R/W
_PCA9955_BIT_CLRERR = const(4)  # W
_PCA9955_BIT_DMBLNK = const(5)  # R/W
_PCA9955_BIT_ERROR = const(6)  # R
_PCA9955_BIT_OVERTEMP = const(7)  # R

_PCA9955_REG_LEDOUT0 = const(0x02)   # R/W
# LEDOUT1 - LEDOUT3 repeats
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED3     |      LED2     |      LED1     |      LEDO     | _PCA9955_REG_LEDOUT0
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED7     |      LED6     |      LED5     |      LED4     | _PCA9955_REG_LEDOUT1
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED11    |      LED10    |      LED9     |      LED8     | _PCA9955_REG_LEDOUT2
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED15    |      LED14    |      LED13    |      LED12    | _PCA9955_REG_LEDOUT3
#---------------------------------------------------------------#
_PCA9955_BIT_LED0 = _PCA9955_BIT_LED4 = _PCA9955_BIT_LED8 = _PCA9955_BIT_LED12 = const(0)  # R/W
_PCA9955_BIT_LED1 = _PCA9955_BIT_LED5 = _PCA9955_BIT_LED9 = _PCA9955_BIT_LED13 = const(2)  # R/W
_PCA9955_BIT_LED2 = _PCA9955_BIT_LED6 = _PCA9955_BIT_LED10 = _PCA9955_BIT_LED14 = const(4)  # R/W
_PCA9955_BIT_LED3 = _PCA9955_BIT_LED7 = _PCA9955_BIT_LED11 = _PCA9955_BIT_LED15 = const(6)  # R/W

_PCA9955_REG_GRPPWM = const(0x06)   # R/W
_PCA9955_REG_GRPFREQ = const(0x07)  # R/W

_PCA9955_REG_PWM0 = const(0x08)   # R/W
# PWM1 - PWM15 repeats

_PCA9955_REG_IREF0 = const(0x18)   # R/W
# IREF1 - IREF15 repeats

_PCA9955_REG_RAMP_RATE_GRP0 = const(0x28)  # R/W
# GRP1 - GRP3 repeats
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
# RAMPUP| RAMPDW|              RAMP RATE                        |
#---------------------------------------------------------------#
_PCA9955_BIT_RAMP_RATE = const(0)  # R/W
_PCA9955_BIT_RAMP_DOWN_ENABLE = const(6)  # R/W
_PCA9955_BIT_RAMP_UP_ENABLE = const(7)  # R/W

_PCA9955_REG_STEP_TIME_GRP0 = const(0x29)  # R/W
# GRP1 - GRP3 repeats
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#   -   |CYCTIME|           FACTOR PER STEP                     |
#---------------------------------------------------------------#
_PCA9955_BIT_FACTOR_PER_STEP = const(0)  # R/W
_PCA9955_BIT_CYCLE_TIME = const(6)  # R/W

_PCA9955_REG_HOLD_CNTL_GRP0 = const(0x2A)  # R/W
# GRP1 - GRP3 repeats
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
# HOLDON|HOLDOFF|      HOLD ON TIME     |     HOLD OFF TIME     |
#---------------------------------------------------------------#
_PCA9955_BIT_HOLD_OFF_TIME = const(0)  # R/W
_PCA9955_BIT_HOLD_ON_TIME = const(3)  # R/W
_PCA9955_BIT_HOLD_OFF_ENABLE = const(6)  # R/W
_PCA9955_BIT_HOLD_ON_ENABLE = const(7)  # R/W

_PCA9955_REG_IREF_GRP0 = const(0x2B)  # R/W
# GRP1 - GRP3 repeats

_PCA9955_REG_GRAD_MODE_SEL0 = const(0x38)  # R/W
_PCA9955_REG_GRAD_MODE_SEL1 = const(0x39)  # R/W

_PCA9955_REG_GRAD_GRP_SEL0 = const(0x3A)  # R/W
# GRP1 - GRP3 repeats
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED3     |      LED2     |      LED1     |      LEDO     | _PCA9955_REG_GRAD_GRP_SEL0
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED7     |      LED6     |      LED5     |      LED4     | _PCA9955_REG_GRAD_GRP_SEL1
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED11    |      LED10    |      LED9     |      LED8     | _PCA9955_REG_GRAD_GRP_SEL2
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED15    |      LED14    |      LED13    |      LED12    | _PCA9955_REG_GRAD_GRP_SEL3
#---------------------------------------------------------------#

_PCA9955_REG_GRAD_CNTL = const(0x3E)   # R/W
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
# START3| CONT3 | START2| CONT2 | START1| CONT1 | START0| CONT0 |
#---------------------------------------------------------------#
_PCA9955_BIT_CONTINUOUS_0 = const(0)  # R/W
_PCA9955_BIT_START_0 = const(1)  # R/W
_PCA9955_BIT_CONTINUOUS_1 = const(2)  # R/W
_PCA9955_BIT_START_1 = const(3)  # R/W
_PCA9955_BIT_CONTINUOUS_2 = const(4)  # R/W
_PCA9955_BIT_START_2 = const(5)  # R/W
_PCA9955_BIT_CONTINUOUS_3 = const(6)  # R/W
_PCA9955_BIT_START_3 = const(7)  # R/W

_PCA9955_REG_OFFSET = const(0x3F)   # R/W
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#               -               |        OUTPUT_DELAY           |
#---------------------------------------------------------------#
_PCA9955_BIT_OUTPUT_DELAY = const(0)  # R/W

_PCA9955_REG_SUBADR1 = const(0x40)  # R/W
_PCA9955_REG_SUBADR2 = const(0x41)  # R/W
_PCA9955_REG_SUBADR3 = const(0x42)  # R/W
_PCA9955_REG_SALLCALLADR = const(0x43)  # R/W
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#                  I2C_BUS_SUBADDRESS                   |   -   |
#---------------------------------------------------------------#
_PCA9955_BIT_SUBADR = const(1)  # R/W

_PCA9955_REG_PWMALL = const(0x44)  # R/W
_PCA9955_REG_IREFALL = const(0x45)  # R/W

_PCA9955_REG_EFLAG0 = const(0x46) # R
# EFLAG1 - EFLAG3 repeats
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED3     |      LED2     |      LED1     |      LEDO     | _PCA9955_REG_EFLAG0
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED7     |      LED6     |      LED5     |      LED4     | _PCA9955_REG_EFLAG1
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED11    |      LED10    |      LED9     |      LED8     | _PCA9955_REG_EFLAG2
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED15    |      LED14    |      LED13    |      LED12    | _PCA9955_REG_EFLAG3
#---------------------------------------------------------------#

# Bit Mask
_1_BIT =  const(0b00000001)
_2_BITS = const(0b00000011)
_3_BITS = const(0b00000111)
_4_BITS = const(0b00001111)
_6_BITS = const(0b00111111)
_7_BITS = const(0b01111111)


class LedChannel:
    """A single PCA9955 channel

    :param PCA9955 device: The PCA9955 device object
    :param int index: The index of the channel
    """
    # User-facing constants:
    NONE = const(0x00)
    SHORT_CIRCUIT = const(0x01)
    OPEN_CIRCUIT = const(0x02)

    OFF = const(0x00)
    FULL_ON = const(0x01)
    PWM = const(0x02)
    PWM_GRP = const(0x03)

    def __init__(self, device: "PCA9955", index: int):
        self._device = device
        self._index = index

    @property
    def brightness(self) -> int:
        """Channel brightness 0 - 255."""
        return self._device._read_8(_PCA9955_REG_PWM0 + self._index)

    @brightness.setter
    def brightness(self, value: int) -> int:
        if not 0 <= value <= 255:
            raise ValueError("Value must be between 0 & 255")
        self._device._write_8(_PCA9955_REG_PWM0 + self._index, value)

    @property
    def gain(self) -> int:
        """Channel curent gain 0 - 255."""
        return self._device._read_8(_PCA9955_REG_IREF0 + self._index)

    @gain.setter
    def gain(self, value: int) -> int:
        if not 0 <= value <= 255:
            raise ValueError("Value must be between 0 & 255")
        self._device._write_8(_PCA9955_REG_IREF0 + self._index, value)

    @property
    def output_state(self) -> int:
        """Channel Driver output state"""

        return self._device.read_channel_config(_PCA9955_REG_LEDOUT0, self._index)

    @output_state.setter
    def output_state(self, value: int) -> int:
        if not LedChannel.OFF <= value <= LedChannel.PWM_GRP:
            raise ValueError(f"Value must be between {LedChannel.OFF} & {LedChannel.PWM_GRP}")
        self._device.write_channel_config(_PCA9955_REG_LEDOUT0, self._index, value)

    @property
    def led_error(self) -> int:
        """LED error state"""
        return self._device.read_channel_config(_PCA9955_REG_EFLAG0, self._index)

    @property
    def group(self) -> int:
        """Gradation group."""
        return self._device.read_channel_config(_PCA9955_REG_GRAD_GRP_SEL0, self._index)

    @group.setter
    def group(self, value: int) -> int:
        if not 0 <= value <= 3:
            raise ValueError(f"Group must be between 0 and 3")
        self._device.write_channel_config(_PCA9955_REG_GRAD_GRP_SEL0, self._index, value)

    @property
    def graduation_mode_select(self) -> bool:
        """1 = grad mode, 0 = normal"""
        offset = self._index % 8
        index =  0 if self._index < 7 else 1
        return bool (self._device.read_register(_PCA9955_REG_GRAD_MODE_SEL0, 
                                                reg_offset = index, 
                                                mask = _1_BIT, 
                                                bit_offset = offset))

    @graduation_mode_select.setter
    def graduation_mode_select(self, value: bool) -> None:
        offset = self._index % 8
        index =  0 if self._index < 7 else 1
        self._device.write_register(_PCA9955_REG_GRAD_MODE_SEL0, 
                                    int(value), 
                                    reg_offset = index, 
                                    mask = _1_BIT, 
                                    bit_offset = offset)


class LedChannels:  # pylint: disable=too-few-public-methods
    """Lazily creates and caches channel objects as needed. Treat it like a sequence.

    :param PCA9955 device: The PCA9955 device object
    """

    def __init__(self, device: "PCA9955") -> None:
        self._device = device
        self._channels = [None] * len(self)

    def __len__(self) -> int:
        return 16

    def __getitem__(self, index: int) -> LedChannel:
        if not self._channels[index]:
            self._channels[index] = LedChannel(self._device, index)
        return self._channels[index]


class Group:
    """A single PCA9685 Graduation Group.

    :param PCA9955 device: The PCA9955 device object
    :param int index: The index of the channel
    """

    def __init__(self, device: "PCA9955", index: int):
        self._device = device
        self._index = index

    @property
    def ramp_up(self) -> bool:
        """Ramp-up enable/disable."""
        return self._device.read_register(_PCA9955_REG_RAMP_RATE_GRP0,
                                          reg_offset = self._index, 
                                          mask = _1_BIT, 
                                          bit_offset =_PCA9955_BIT_RAMP_UP_ENABLE)

    @ramp_up.setter
    def ramp_up(self, value: bool) -> bool:
        self._device.write_register(_PCA9955_REG_RAMP_RATE_GRP0, 
                                    value, 
                                    reg_offset = self._index, 
                                    mask = _1_BIT, 
                                    bit_offset = _PCA9955_BIT_RAMP_UP_ENABLE)

    @property
    def ramp_down(self) -> bool:
        """Ramp-down enable/disable."""
        return self._device.read_register(_PCA9955_REG_RAMP_RATE_GRP0,  
                                          reg_offset = self._index, 
                                          mask = _1_BIT, 
                                          bit_offset = _PCA9955_BIT_RAMP_DOWN_ENABLE)

    @ramp_down.setter
    def ramp_down(self, value: bool) -> bool:
        self._device.write_register(_PCA9955_REG_RAMP_RATE_GRP0, 
                                    value, 
                                    reg_offset = self._index, 
                                    mask = _1_BIT, 
                                    bit_offset = _PCA9955_BIT_RAMP_DOWN_ENABLE)

    @property
    def ramp_rate(self) -> int:
        """Ramp rate per step 0 - 64."""
        return self._device.read_register(_PCA9955_REG_RAMP_RATE_GRP0, 
                                          reg_offset = self._index, 
                                          mask = _6_BITS, 
                                          bit_offset = _PCA9955_BIT_RAMP_RATE)

    @ramp_rate.setter
    def ramp_rate(self, value: int) -> int:
        if not 0 <= value <= 64:
            raise ValueError("Value must be between 0 & 64")
        self._device.write_register(_PCA9955_REG_RAMP_RATE_GRP0, 
                                    value, 
                                    reg_offset = self._index, 
                                    mask = _6_BITS, 
                                    bit_offset = _PCA9955_BIT_RAMP_RATE)

    @property
    def cycle_time(self) -> int:
        """Cycle time - 0 (0.5ms) or 1 (8ms)."""
        return self._device.read_register(_PCA9955_REG_STEP_TIME_GRP0 , 
                                          reg_offset = self._index, 
                                          mask = _1_BIT, 
                                          bit_offset = _PCA9955_BIT_CYCLE_TIME)

    @cycle_time.setter
    def cycle_time(self, value: int) -> int:
        if not 0 <= value <= 1:
            raise ValueError("Valid values are 0 (0.5ms) or 1 (8ms)")
        self._device.write_register(_PCA9955_REG_STEP_TIME_GRP0, 
                                    value, 
                                    reg_offset = self._index, 
                                    mask = _1_BIT, 
                                    bit_offset = _PCA9955_BIT_CYCLE_TIME)

    @property
    def factor_per_step(self) -> int:
        """Multiple factor per step 0 - 64."""
        return self._device.read_register(_PCA9955_REG_STEP_TIME_GRP0, 
                                          reg_offset = self._index, 
                                          mask =_6_BITS, 
                                          bit_offset = _PCA9955_BIT_FACTOR_PER_STEP)

    @factor_per_step.setter
    def factor_per_step(self, value: int) -> int:
        if not 0 <= value <= 64:
            raise ValueError("Value must be between 0 & 64")
        self._device.write_register(_PCA9955_REG_STEP_TIME_GRP0, 
                                    value, 
                                    reg_offset = self._index, 
                                    mask = _6_BITS, 
                                    bit_offset = _PCA9955_BIT_FACTOR_PER_STEP)

    @property
    def hold_on(self) -> bool:
        """Hold on enable/disable."""
        return self._device.read_register(_PCA9955_REG_HOLD_CNTL_GRP0, 
                                          reg_offset = self._index, 
                                          mask = _1_BIT, 
                                          bit_offset = _PCA9955_BIT_HOLD_ON_ENABLE)

    @hold_on.setter
    def hold_on(self, value: bool) -> bool:
        self._device.write_register(_PCA9955_REG_HOLD_CNTL_GRP0, 
                                    value, 
                                    reg_offset = self._index, 
                                    mask = _1_BIT, 
                                    bit_offset =_PCA9955_BIT_HOLD_ON_ENABLE)

    @property
    def hold_off(self) -> bool:
        """Hold off enable/disable."""
        return self._device.read_register(_PCA9955_REG_HOLD_CNTL_GRP0, 
                                          reg_offset = self._index, 
                                          mask =_1_BIT, 
                                          bit_offset = _PCA9955_BIT_HOLD_OFF_ENABLE)

    @hold_off.setter
    def hold_off(self, value: bool) -> bool:
        self._device.write_register(_PCA9955_REG_HOLD_CNTL_GRP0, 
                                    value, 
                                    reg_offset = self._index, 
                                    mask = _1_BIT, 
                                    bit_offset = _PCA9955_BIT_HOLD_OFF_ENABLE)

    @property
    def hold_on_time(self) -> int:
        """Hold On time - 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s), 4 (1s), 5 (2s), 6 (4s), 7 (6s)."""
        return self._device.read_register(_PCA9955_REG_HOLD_CNTL_GRP0, 
                                          reg_offset = self._index, 
                                          mask = _3_BITS, 
                                          bit_offset = _PCA9955_BIT_HOLD_ON_TIME)

    @hold_on_time.setter
    def hold_on_time(self, value: int) -> int:
        if not 0 <= value <= 7:
            raise ValueError("Valid values are 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s), 4 (1s), 5 (2s), 6 (4s), 7 (6s)")
        self._device.write_register(_PCA9955_REG_HOLD_CNTL_GRP0, 
                                    value, 
                                    reg_offset = self._index, 
                                    mask = _3_BITS, 
                                    bit_offset =_PCA9955_BIT_HOLD_ON_TIME)

    @property
    def hold_off_time(self) -> int:
        """Hold On time  - 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s), 4 (1s), 5 (2s), 6 (4s), 7 (6s)."""
        return self._device.read_register(_PCA9955_REG_HOLD_CNTL_GRP0, 
                                          reg_offset = self._index, 
                                          mask = _3_BITS, 
                                          bit_offset = _PCA9955_BIT_HOLD_OFF_TIME)

    @hold_off_time.setter
    def hold_off_time(self, value: int) -> int:
        if not 0 <= value <= 7:
            raise ValueError("Valid values are 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s), 4 (1s), 5 (2s), 6 (4s), 7 (6s)")
        self._device.write_register(_PCA9955_REG_HOLD_CNTL_GRP0, 
                                    value, 
                                    reg_offset = self._index, 
                                    mask = _3_BITS, 
                                    bit_offset = _PCA9955_BIT_HOLD_OFF_TIME)

    @property
    def gain(self) -> int:
        """Group output current gain (0-255)."""
        return self._device.read_register(_PCA9955_REG_IREF_GRP0, 
                                          self._index)

    @gain.setter
    def gain(self, value: int) -> int:
        """Set group output current gain to specified value. Should be 0-255."""
        if not 0 <= value <= 255:
            raise ValueError("Valid values are  0-255")
        self._device.write_register(_PCA9955_REG_IREF_GRP0, 
                                    value, 
                                    reg_offset = self._index)

    @property
    def graduation_mode(self) -> bool:
        """Graduation mode. 0 = Single Shot, 1 = Continuous."""
        bitOffset = self._index << 2
        return bool(self._device.read_register(_PCA9955_REG_GRAD_CNTL, 
                                               mask = _1_BIT, 
                                               bit_offset = bitOffset))

    @graduation_mode.setter
    def graduation_mode(self, value: bool) -> None:
        """Set graduation mode. 0 = Single Shot, 1 = Continuous."""
        bitOffset = self._index << 2
        self._device.write_register(_PCA9955_REG_GRAD_CNTL, 
                                    int(value), 
                                    mask = _1_BIT, 
                                    bit_offset = bitOffset)

    def graduation_start(self) -> None:
        bitOffset = (self._index << 2) + 1
        self._device.write_register(_PCA9955_REG_GRAD_CNTL, 0x01, 
                                    mask = _1_BIT, 
                                    bit_offset = bitOffset)

    def graduation_stop(self) -> None:
        bitOffset = (self._index << 2) + 1
        self._device.write_register(_PCA9955_REG_GRAD_CNTL, 
                                    0x00, 
                                    mask = _1_BIT, 
                                    bit_offset = bitOffset)


class Groups:  # pylint: disable=too-few-public-methods
    """Lazily creates and caches Group objects as needed. Treat it like a sequence.

    :param PCA9955 device: The PCA9955 device object
    """

    def __init__(self, device: "PCA9955") -> None:
        self._device = device
        self.groups = [None] * len(self)

    def __len__(self) -> int:
        return 4

    def __getitem__(self, index: int) -> Group:
        if not self.groups[index]:
            self.groups[index] = Group(self._device, index)
        return self.groups[index]


class PCA9955:
    """
    Initialise the PCA9955 chip at ``address`` on ``i2c_bus``.

    :param ~busio.I2C i2c_bus: The I2C bus which the PCA9955 is connected to.
    :param int address: The I2C address of the PCA9955.
    :param int reference_clock_speed: The frequency of the internal reference clock in Hertz.
    """

    def __init__(self,
                 i2c: I2C,
                 address: int = _PCA9955B_DEFAULT_I2C_ADDR,
                 oe_pin:Optional[microcontroller.Pin] = None,
                 reset_pin:Optional[microcontroller.Pin] = None) -> None:

        self._device = i2c_device.I2CDevice(i2c, address)
        self.channels = LedChannels(self)
        self.groups = Groups(self)
        self._oe = None
        if oe_pin is not None:
            self._oe = DigitalInOut(oe_pin)
            self._oe.direction = Direction.OUTPUT
            self._oe.drive_mode = DriveMode.OPEN_DRAIN
        self._reset =  None
        if reset_pin is not None:
            self._reset = DigitalInOut(reset_pin)
            self._reset.direction = Direction.OUTPUT
            self._reset.drive_mode = DriveMode.OPEN_DRAIN
            self._reset.value = True

    def __enter__(self) -> "PCA9955":
        return self

    def __exit__(
        self,
        exception_type: Optional[Type[type]],
        exception_value: Optional[BaseException],
        traceback: Optional[TracebackType],
        ) -> None:
        return False

    def deinit(self) -> None:
        """Stop using the PCA9955."""

    def reset(self) -> None:
        if self._reset is not None:
            self._reset.value = False
            time.sleep(.01)
            self._reset.value = True

    @property
    def oe(self) -> Optional[bool]:
        if self._oe is not None:
            return not self._oe.value
        else:
            return None

    @oe.setter
    def oe(self, value:bool) -> None:
        if self._oe is not None:
            self._oe.value = not value

    @property
    def brightness(self) -> NoReturn:
        """Global brightness 0 - 255."""
        raise AttributeError("brightness is write-only")

    @brightness.setter
    def brightness(self, value: int) -> None:
        self._write_8(_PCA9955_REG_PWMALL, value)

    @property
    def gain(self) -> NoReturn:
        """Global output currrent 0 - 255."""
        raise AttributeError("Output current is write-only")

    @gain.setter
    def gain(self, value: int) -> int:
        self._write_8(_PCA9955_REG_IREFALL, value)

    @property
    def over_temp(self) -> bool:
        """True indicates over temperature condition."""
        return bool(self.read_register(_PCA9955_REG_MODE2, 
                                       mask = _1_BIT, 
                                       bit_offset = _PCA9955_BIT_OVERTEMP))

    @over_temp.setter
    def over_temp(self) -> NoReturn:
        raise AttributeError("Over Temp is read-only")

    @property
    def errors_exist(self) -> bool:
        """True indicates errors exist."""
        return bool(self.read_register(_PCA9955_REG_MODE2, 
                                       mask = _1_BIT, 
                                       bit_offset = _PCA9955_BIT_ERROR))

    @errors_exist.setter
    def errors_exist(self) -> NoReturn:
        raise AttributeError("Errors Exist is read-only")

    @property
    def low_power_mode(self) -> bool:
        return bool(self.read_register(_PCA9955_REG_MODE1, 
                                       mask = _1_BIT, 
                                       bit_offset = _PCA9955_BIT_SLEEP))

    @low_power_mode.setter
    def low_power_mode(self, value: bool) -> None:
        self.write_register(_PCA9955_REG_MODE1, 
                            value, 
                            mask = _1_BIT, 
                            bit_offset =_PCA9955_BIT_SLEEP)

    @property
    def auto_increment_flag(self) -> bool:
        return bool(self.read_register(_PCA9955_REG_MODE1, 
                                       mask = _1_BIT, 
                                       bit_offset = _PCA9955_BIT_AIF))

    @auto_increment_flag.setter
    def auto_increment_flag(self, value: bool) -> NoReturn:
        raise AttributeError("AIF is read-only")

    @property
    def auto_increment_mode(self) -> int:
        return self.read_register(_PCA9955_REG_MODE1, 
                                  mask = _2_BITS, 
                                  bit_offset = _PCA9955_BIT_AI0)

    @auto_increment_mode.setter
    def auto_increment_mode(self, value: int) -> None:
        if not 0 <= value <= 3:
            raise ValueError("Valid values are 0 - 3")
        self.write_register(_PCA9955_REG_MODE1, 
                            value, 
                            mask = _2_BITS, 
                            bit_offset =_PCA9955_BIT_AI0)

    @property
    def exponential_graduation(self) -> bool:
        """1 = exponential adjustment for gradation control, 0 = linear adjustment for gradation control (default)"""
        return bool(self.read_register(_PCA9955_REG_MODE2,
                                       mask = _1_BIT, 
                                       bit_offset = _PCA9955_BIT_EXP_EN))

    @exponential_graduation.setter
    def exponential_graduation(self, value: bool) -> None:
        self.write_register(_PCA9955_REG_MODE2, int(value), 
                            mask = _1_BIT, 
                            bit_offset =_PCA9955_BIT_EXP_EN)

    @property
    def group_blinking(self) -> bool:
        """1 = group control - blinking, 0 =  group control - dimming (default)"""
        return bool(self.read_register(_PCA9955_REG_MODE2, 
                                       mask = _1_BIT, 
                                       bit_offset = _PCA9955_BIT_DMBLNK))

    @group_blinking.setter
    def group_blinking(self, value: bool) -> None:
        self.write_register(_PCA9955_REG_MODE2, int(value), 
                            mask = _1_BIT, 
                            bit_offset =_PCA9955_BIT_DMBLNK)

    @property
    def group_pwm(self) -> int:
        """Global brightness control when group_blinking = 0)"""
        return self._read_8(_PCA9955_REG_GRPPWM)

    @group_pwm.setter
    def group_pwm(self, value: int) -> None:
        self._write_8(_PCA9955_REG_GRPPWM, value)

    @property
    def group_frequency(self) -> int:
        """Global blinking frequency  control when group_blinking = 1)"""
        return self._read_8(_PCA9955_REG_GRPFREQ)

    @group_frequency.setter
    def group_frequency(self, value: int) -> None:
        self._write_8(_PCA9955_REG_GRPFREQ, value)

    def clear_errors(self) -> None:
        self.write_register(_PCA9955_REG_MODE2, 
                            0x01, 
                            mask = _1_BIT, 
                            bit_offset = _PCA9955_BIT_CLRERR)

    @property
    def output_delay_offset(self) -> int:
        return self.read_register(_PCA9955_REG_OFFSET, 
                                  mask = _4_BITS, 
                                  bit_offset = _PCA9955_BIT_OUTPUT_DELAY)

    @output_delay_offset.setter
    def output_delay_offset(self, value: int) -> None:
        if value not in [0x00, 0x01, 0x02, 0x03, 0x0F]:
            raise ValueError(f"Value must be one of {0x00:00x}, {0x01:02x}, {0x02:02x}, {0x03:02x} or{0x0F:02x}")
        self.write_register(_PCA9955_REG_OFFSET, 
                            value, 
                            mask = _4_BITS, 
                            bit_offset =_PCA9955_BIT_OUTPUT_DELAY)


    # I2C Sub Address constants
    SUBADR1 = const(0) # offset from _PCA9955_REG_SUBADR1
    SUBADR2 = const(1)
    SUBADR3 = const(2)
    ALLCALLADR = const(3)

    def get_i2c_address(self, addr:int) -> int:
        if addr not in [PCA9955.SUBADR1, PCA9955.SUBADR2, PCA9955.SUBADR3, PCA9955.ALLCALLADR ]:
            raise ValueError(f"Value must be one of {PCA9955.SUBADR1}, {PCA9955.SUBADR2}, {PCA9955.SUBADR3} or {PCA9955.ALLCALLADR}")
        return int(self.read_register(_PCA9955_REG_SUBADR1, 
                                      reg_offset = addr, 
                                      mask = _7_BITS, 
                                      bit_offset = _PCA9955_BIT_SUBADR))

    def set_i2c_address(self, addr:int, value: int) -> None:
        if addr not in [PCA9955.SUBADR1, PCA9955.SUBADR2, PCA9955.SUBADR3, PCA9955.ALLCALLADR ]:
            raise ValueError(f"Value must be one of {PCA9955.SUBADR1}, {PCA9955.SUBADR2}, {PCA9955.SUBADR3} or {PCA9955.ALLCALLADR}")
        self.write_register(_PCA9955_REG_SUBADR1, 
                            value, 
                            reg_offset = addr, 
                            mask = _7_BITS, 
                            bit_offset = _PCA9955_BIT_SUBADR)

    def enable_i2c_address(self, addr:int, value: bool) -> None:
        if addr not in [PCA9955.SUBADR1, PCA9955.SUBADR2, PCA9955.SUBADR3, PCA9955.ALLCALLADR ]:
            raise ValueError(f"Value must be one of {PCA9955.SUBADR1}, {PCA9955.SUBADR2}, {PCA9955.SUBADR3} or {PCA9955.ALLCALLADR}")
        bitOffset = PCA9955.ALLCALLADR - addr # Bits in reverse order to registers
        self.write_register(_PCA9955_REG_MODE1, int(value), 
                            mask = _1_BIT, 
                            bit_offset = bitOffset)

    def is_i2c_address_enabled(self, addr:int, value: int) -> bool:
        if addr not in [PCA9955.SUBADR1, PCA9955.SUBADR2, PCA9955.SUBADR3, PCA9955.ALLCALLADR ]:
            raise ValueError(f"Value must be one of {PCA9955.SUBADR1}, {PCA9955.SUBADR2}, {PCA9955.SUBADR3} or {PCA9955.ALLCALLADR}")
        bitOffset = PCA9955.ALLCALLADR - addr # Bits in reverse order to registers
        return bool(self.read_register(_PCA9955_REG_MODE1, 
                                       mask = _1_BIT, 
                                       bit_offset = bitOffset))


    def read_register(self, base_register: int, reg_offset: int = 0, mask: int = 0xFF, bit_offset: int = 0) -> int:
        """ Read set of bits from register"""
        register = base_register + reg_offset
        mask = mask << bit_offset
        reg = self._read_8(register)
        result = (reg & mask) >> bit_offset
        print(f"rr - reg:{register:#x} offset:{bit_offset:#x} reg: {reg:08b} mask:{mask:08b} value:0x{result:02x}")
        return result

    def write_register(self, base_register: int, value: int, reg_offset: int = 0, mask: int = 0xFF, bit_offset: int = 0) -> None:
        """ Write set of bits to register"""
        register = base_register + reg_offset
        mask = mask << bit_offset
        inverse_mask = ~mask & 0xFF
        reg = self._read_8(register)
        value = (reg & inverse_mask) | (value << bit_offset)
        self._write_8(register, value)
        print(f"wr - reg:{register:#x} offset:{bit_offset:#x} reg: {reg:08b} mask:{mask:08b} value:0x{value:02x}")

    def read_channel_config(self, base_register: int, reg_offset: int) -> int:
        """ Read channel configuration register"""
        register = base_register + (reg_offset >> 2)
        offset = (reg_offset % 4) << 1
        mask = _2_BITS << offset
        reg = self._read_8(register)
        result = (reg & mask) >> offset
        print(f"rc - reg:{register:#x} offset:{offset:#x} reg: {reg:08b} mask:{mask:08b} value:0x{result:02x}")
        return result

    def write_channel_config(self, base_register: int, reg_offset: int, value: int) -> None:
        """ Write channel configuration register"""
        register = base_register + (reg_offset >> 2)
        offset = (reg_offset % 4) << 1
        mask = _2_BITS << offset
        inverse_mask = ~mask & 0xFF
        reg = self._read_8(register)
        value = (reg & inverse_mask) | (value << offset)
        self._write_8(register, value)
        print(f"wc - reg:{register:#x} offset:{offset:#x} reg: {reg:08b} mask:{mask:08b} value:0x{value:02x}")

    def _read_8(self, address: int) -> int:
        """ Read and return a byte from the specified 8-bit register address."""
        result = bytearray(1)
        with self._device as i2c:
            i2c.write(bytes([address]))
            i2c.readinto(result)
        print(f"r8 - address:{address:#x} value:{result[0]:08b}")
        return result[0]

    def _write_8(self, address: int, value: int) -> None:
        """ Write a byte to the specified 8-bit register address."""
        result = bytearray(1)
        with self._device as i2c:
            i2c.write(bytes([address, value]))
            i2c.write(bytes([address]))
            i2c.readinto(result)
        print(f"w8 - address:{address:#x} value:{value:08b}")
