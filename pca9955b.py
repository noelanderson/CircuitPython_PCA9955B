# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
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

_PCA9955B_DEFAULT_I2C_ADDR = const(0x3F) # AD10 AD1 & AD2 all FLT (Floating Inputs)

# Register map & bit positions

_REGISTER_MODE1 = const(0x00)   # R/W
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#  AIF  |  AI1  |  AI0  | SLEEP |  SUB1 |  SUB2 |  SUB3 |ALLCALL|
#---------------------------------------------------------------#
_BIT_POS_ALLCALL = const(0)  #R
_BIT_POS_SUB3 = const(1)  # R/W
_BIT_POS_SUB2 = const(2)  # R/W
_BIT_POS_SUB1 = const(3)  # R/W
_BIT_POS_SLEEP = const(4)  # R/W
_BIT_POS_AI0 = const(5)  # R/W
_BIT_POS_AI1 = const(6)  # R/W
_BIT_POS_AIF = const(7)  # R/W

_REGISTER_MODE2 = const(0x01)   # R/W
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
# OVERTP| ERROR | DMBLNK| CLRERR|  OCH  |EXP_EN |   -   |   -   |
#---------------------------------------------------------------#
_BIT_POS_EXP_EN = const(2)  # R/W
_BIT_POS_OCH = const(3)  # R/W
_BIT_POS_CLRERR = const(4)  # W
_BIT_POS_DMBLNK = const(5)  # R/W
_BIT_POS_ERROR = const(6)  # R
_BIT_POS_OVERTEMP = const(7)  # R

_REGISTER_LEDOUT0 = const(0x02)   # R/W
# LEDOUT1 - LEDOUT3 repeats
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED3     |      LED2     |      LED1     |      LEDO     | _REGISTER_LEDOUT0 
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED7     |      LED6     |      LED5     |      LED4     | _REGISTER_LEDOUT1
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED11    |      LED10    |      LED9     |      LED8     | _REGISTER_LEDOUT2
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED15    |      LED14    |      LED13    |      LED12    | _REGISTER_LEDOUT3
#---------------------------------------------------------------#
_BIT_POS_LED0 = _BIT_POS_LED4 = _BIT_POS_LED8 = _BIT_POS_LED12 = const(0)  # R/W
_BIT_POS_LED1 = _BIT_POS_LED5 = _BIT_POS_LED9 = _BIT_POS_LED13 = const(2)  # R/W
_BIT_POS_LED2 = _BIT_POS_LED6 = _BIT_POS_LED10 = _BIT_POS_LED14 = const(4)  # R/W
_BIT_POS_LED3 = _BIT_POS_LED7 = _BIT_POS_LED11 = _BIT_POS_LED15 = const(6)  # R/W

_REGISTER_GRPPWM = const(0x06)   # R/W
_REGISTER_GRPFREQ = const(0x07)  # R/W

_REGISTER_PWM0 = const(0x08)   # R/W 
# PWM1 - PWM15 repeats


_REGISTER_IREF0 = const(0x18)   # R/W
# IREF1 - IREF15 repeats

_REGISTER_RAMP_RATE_GRP0 = const(0x28)  # R/W
# GRP1 - GRP3 repeats
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
# RAMPUP| RAMPDW|              RAMP RATE                        |
#---------------------------------------------------------------#
_BIT_POS_RAMP_RATE = const(0)  # R/W
_BIT_POS_RAMP_DOWN_ENABLE = const(6)  # R/W
_BIT_POS_RAMP_UP_ENABLE = const(7)  # R/W

_REGISTER_STEP_TIME_GRP0 = const(0x29)  # R/W
# GRP1 - GRP3 repeats
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#   -   |CYCTIME|           FACTOR PER STEP                     |
#---------------------------------------------------------------#
_BIT_POS_FACTOR_PER_STEP = const(0)  # R/W
_BIT_POS_CYCLE_TIME = const(6)  # R/W

_REGISTER_HOLD_CNTL_GRP0 = const(0x2A)  # R/W
# GRP1 - GRP3 repeats
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
# HOLDON|HOLDOFF|      HOLD ON TIME     |     HOLD OFF TIME     |
#---------------------------------------------------------------#
_BIT_POS_HOLD_OFF_TIME = const(0)  # R/W
_BIT_POS_HOLD_ON_TIME = const(3)  # R/W
_BIT_POS_HOLD_OFF_ENABLE = const(6)  # R/W
_BIT_POS_HOLD_ON_ENABLE = const(7)  # R/W

_REGISTER_IREF_GRP0 = const(0x2B)  # R/W
# GRP1 - GRP3 repeats

_REGISTER_GRAD_MODE_SEL0 = const(0x38)  # R/W
_REGISTER_GRAD_MODE_SEL1 = const(0x39)  # R/W

_REGISTER_GRAD_GRP_SEL0 = const(0x3A)  # R/W
# GRP1 - GRP3 repeats
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED3     |      LED2     |      LED1     |      LEDO     | _REGISTER_GRAD_GRP_SEL0 
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED7     |      LED6     |      LED5     |      LED4     | _REGISTER_GRAD_GRP_SEL1
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED11    |      LED10    |      LED9     |      LED8     | _REGISTER_GRAD_GRP_SEL2
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED15    |      LED14    |      LED13    |      LED12    | _REGISTER_GRAD_GRP_SEL3
#---------------------------------------------------------------#

_REGISTER_GRAD_CNTL = const(0x3E)   # R/W
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
# START3| CONT3 | START2| CONT2 | START1| CONT1 | START0| CONT0 |
#---------------------------------------------------------------#
_BIT_POS_CONTINUOUS_0 = const(0)  # R/W
_BIT_POS_START_0 = const(1)  # R/W
_BIT_POS_CONTINUOUS_1 = const(2)  # R/W
_BIT_POS_START_1 = const(3)  # R/W
_BIT_POS_CONTINUOUS_2 = const(4)  # R/W
_BIT_POS_START_2 = const(5)  # R/W
_BIT_POS_CONTINUOUS_3 = const(6)  # R/W
_BIT_POS_START_3 = const(7)  # R/W

_REGISTER_OFFSET = const(0x3F)   # R/W
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#               -               |        OUTPUT_DELAY           |
#---------------------------------------------------------------#
_BIT_POS_OUTPUT_DELAY = const(0)  # R/W

_REGISTER_SUBADR1 = const(0x40)  # R/W
_REGISTER_SUBADR2 = const(0x41)  # R/W
_REGISTER_SUBADR3 = const(0x42)  # R/W
_REGISTER_SALLCALLADR = const(0x43)  # R/W
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#                  I2C_BUS_SUBADDRESS                   |   -   |
#---------------------------------------------------------------#
_BIT_POS_SUBADR = const(1)  # R/W

_REGISTER_PWMALL = const(0x44)  # R/W
_REGISTER_IREFALL = const(0x45)  # R/W

_REGISTER_EFLAG0 = const(0x46) # R
# EFLAG1 - EFLAG3 repeats
#---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED3     |      LED2     |      LED1     |      LEDO     | _REGISTER_EFLAG0 
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED7     |      LED6     |      LED5     |      LED4     | _REGISTER_EFLAG1
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED11    |      LED10    |      LED9     |      LED8     | _REGISTER_EFLAG2
#-------+-------+-------+-------+-------+-------+-------+-------|
#      LED15    |      LED14    |      LED13    |      LED12    | _REGISTER_EFLAG3
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
    LED_ERROR_NONE = const(0x00)
    LED_ERROR_SHORT_CIRCUIT = const(0x01)
    LED_ERROR_OPEN_CIRCUIT = const(0x02)

    LED_DRIVER_OFF = const(0x00)
    LED_DRIVER_FULL_ON = const(0x01)
    LED_DRIVER_PWM = const(0x02)
    LED_DRIVER_PWM_GRP = const(0x03)

    def __init__(self, device: "PCA9955", index: int):
        self._device = device
        self._index = index

    @property
    def brightness(self) -> int:
        """Channel brightness 0 - 255."""
        return self._device._read_8(_REGISTER_PWM0 + self._index)

    @brightness.setter
    def brightness(self, value: int) -> int:
        if not 0 <= value <= 255:
            raise ValueError("Value must be between 0 & 255")
        self._device._write_8(_REGISTER_PWM0 + self._index, value)

    @property
    def gain(self) -> int:
        """Channel curent gain 0 - 255."""
        return self._device._read_8(_REGISTER_IREF0 + self._index)

    @gain.setter
    def gain(self, value: int) -> int:
        if not 0 <= value <= 255:
            raise ValueError("Value must be between 0 & 255")
        self._device._write_8(_REGISTER_IREF0 + self._index, value)

    @property
    def output_state(self) -> int:
        """Channel Driver output state"""
        return self._device.read_channel_config(_REGISTER_LEDOUT0, self._index)

    @output_state.setter
    def output_state(self, value: int) -> int:
        if not LedChannel.LED_DRIVER_OFF <= value <= LedChannel.LED_DRIVER_PWM_GRP:
            raise ValueError(f"Value must be between {LedChannel.LED_DRIVER_OFF} & {LedChannel.LED_DRIVER_PWM_GRP}")
        self._device.write_channel_config(_REGISTER_LEDOUT0, self._index, value)

    @property
    def led_error(self) -> int:
        """LED error state"""
        return self._device.read_channel_config(_REGISTER_EFLAG0, self._index)

    @property
    def group(self) -> int:
        """Gradation group."""
        return self._device.read_channel_config(_REGISTER_GRAD_GRP_SEL0, self._index)

    @group.setter
    def group(self, value: int) -> int:
        if not 0 <= value <= 3:
            raise ValueError(f"Group must be between 0 and 3")
        self._device.write_channel_config(_REGISTER_GRAD_GRP_SEL0, self._index, value)

    @property
    def graduation_mode_select(self) -> bool:
        """1 = grad mode, 0 = normal"""
        offset = self._index % 8
        index =  0 if self.index < 7 else 1
        return bool (self._device.read_register(_REGISTER_GRAD_MODE_SEL0, index = index, mask = _1_BIT, offset = offset))

    @graduation_mode_select.setter
    def graduation_mode_select(self, value: bool) -> None:
        offset = self._index % 8
        index =  0 if self.index < 7 else 1
        self._device.write_register(_REGISTER_GRAD_MODE_SEL0, int(value), index = index, mask = _1_BIT, offset = offset)


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
    """A single PCA9685 cGraduation Group.

    :param PCA9955 device: The PCA9955 device object
    :param int index: The index of the channel
    """

    def __init__(self, device: "PCA9955", index: int):
        self._device = device
        self._index = index

    @property
    def ramp_up(self) -> bool:
        """Ramp-up enable/disable."""
        return self._device.read_register(_REGISTER_RAMP_RATE_GRP0, index = self._index, mask = _1_BIT, offset =_BIT_POS_RAMP_UP_ENABLE)

    @ramp_up.setter
    def ramp_up(self, value: bool) -> bool:
        self._device.write_register(_REGISTER_RAMP_RATE_GRP0, value, index = self._index, mask = _1_BIT, offset = _BIT_POS_RAMP_UP_ENABLE)

    @property
    def ramp_down(self) -> bool:
        """Ramp-down enable/disable."""
        return self._device.read_register(_REGISTER_RAMP_RATE_GRP0,  index = self._index, mask = _1_BIT, offset = _BIT_POS_RAMP_DOWN_ENABLE)

    @ramp_down.setter
    def ramp_down(self, value: bool) -> bool:
        self._device.write_register(_REGISTER_RAMP_RATE_GRP0, value, index = self._index, mask = _1_BIT, offset = _BIT_POS_RAMP_DOWN_ENABLE)

    @property
    def ramp_rate(self) -> int:
        """Ramp rate per step 0 - 64."""
        return self._device.read_register(_REGISTER_RAMP_RATE_GRP0, index = self._index, mask = _6_BITS, offset = _BIT_POS_RAMP_RATE)

    @ramp_rate.setter
    def ramp_rate(self, value: int) -> int:
        if not 0 <= value <= 64:
            raise ValueError("Value must be between 0 & 64")
        self._device.write_register(_REGISTER_RAMP_RATE_GRP0, value, index = self._index, mask = _6_BITS, offset = _BIT_POS_RAMP_RATE)

    @property
    def cycle_time(self) -> int:
        """Cycle time - 0 (0.5ms) or 1 (8ms)."""
        return self._device.read_register(_REGISTER_STEP_TIME_GRP0 , index = self._index, mask = _1_BIT, offset = _BIT_POS_CYCLE_TIME)

    @cycle_time.setter
    def cycle_time(self, value: int) -> int:
        if not 0 <= value <= 1:
            raise ValueError("Valid values are 0 (0.5ms) or 1 (8ms)")
        self._device.write_register(_REGISTER_STEP_TIME_GRP0, value, index = self._index, mask = _1_BIT, offset = _BIT_POS_CYCLE_TIME)

    @property
    def factor_per_step(self) -> int:
        """Multiple factor per step 0 - 64."""
        return self._device.read_register(_REGISTER_STEP_TIME_GRP0, index = self._index, mask =_6_BITS, offset = _BIT_POS_FACTOR_PER_STEP)

    @factor_per_step.setter
    def factor_per_step(self, value: int) -> int:
        if not 0 <= value <= 64:
            raise ValueError("Value must be between 0 & 64")
        self._device.write_register(_REGISTER_STEP_TIME_GRP0, value, index = self._index, mask = _6_BITS, offset = _BIT_POS_FACTOR_PER_STEP)

    @property
    def hold_on(self) -> bool:
        """Hold on enable/disable."""
        return self._device.read_register(_REGISTER_HOLD_CNTL_GRP0, index = self._index, mask = _1_BIT, offset = _BIT_POS_HOLD_ON_ENABLE)

    @hold_on.setter
    def hold_on(self, value: bool) -> bool:
        self._device.write_register(_REGISTER_HOLD_CNTL_GRP0, value, index = self._index, mask = _1_BIT, offset =_BIT_POS_HOLD_ON_ENABLE)

    @property
    def hold_off(self) -> bool:
        """Hold off enable/disable."""
        return self._device.read_register(_REGISTER_HOLD_CNTL_GRP0, index = self._index, mask =_1_BIT, offset = _BIT_POS_HOLD_OFF_ENABLE)

    @hold_off.setter
    def hold_off(self, value: bool) -> bool:
        self._device.write_register(_REGISTER_HOLD_CNTL_GRP0, value, index = self._index, mask = _1_BIT, offset = _BIT_POS_HOLD_OFF_ENABLE)

    @property
    def hold_on_time(self) -> int:
        """Hold On time - 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s), 4 (1s), 5 (2s), 6 (4s), 7 (6s)."""
        return self._device.read_register(_REGISTER_HOLD_CNTL_GRP0, index = self._index, mask = _3_BITS, offset = _BIT_POS_HOLD_ON_TIME)

    @hold_on_time.setter
    def hold_on_time(self, value: int) -> int:
        if not 0 <= value <= 64:
            raise ValueError("Valid values are 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s), 4 (1s), 5 (2s), 6 (4s), 7 (6s)")
        self._device.write_register(_REGISTER_HOLD_CNTL_GRP0, value, index = self._index, mask = _3_BITS, offset =_BIT_POS_HOLD_ON_TIME)

    @property
    def hold_off_time(self) -> int:
        """Hold On time  - 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s), 4 (1s), 5 (2s), 6 (4s), 7 (6s)."""
        return self._device.read_register(_REGISTER_HOLD_CNTL_GRP0, index = self._index, mask = _3_BITS, offset = _BIT_POS_HOLD_OFF_TIME)

    @hold_off_time.setter
    def hold_off_time(self, value: int) -> int:
        if not 0 <= value <= 64:
            raise ValueError("Valid values are 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s), 4 (1s), 5 (2s), 6 (4s), 7 (6s)")
        self._device.write_register(_REGISTER_HOLD_CNTL_GRP0, value, index = self._index, mask = _3_BITS, offset = _BIT_POS_HOLD_OFF_TIME)

    @property
    def output_gain_control(self) -> int:
        """Output current gain 0-255."""
        return self._device.read_register(_REGISTER_IREF_GRP0, self._index)

    @output_gain_control.setter
    def output_gain_control(self, value: int) -> int:
        if not 0 <= value <= 255:
            raise ValueError("Valid values are  0-255")
        self._device.write_register(_REGISTER_IREF_GRP0, value, index = self._index)


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

    def __init__(self, i2c: I2C, address: int = _PCA9955B_DEFAULT_I2C_ADDR) -> None:
        self._device = i2c_device.I2CDevice(i2c, address)
        self.channels = LedChannels(self)
        self.groups = Groups(self)

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

    @property
    def brightness(self) -> NoReturn:
        """Global brightness 0 - 255.""" 
        raise AttributeError("brightness is write-only")
    
    @brightness.setter
    def brightness(self, value: int) -> None:
        self._write_8(_REGISTER_PWMALL, value)

    @property
    def output_current(self) -> NoReturn:
        """Global output currrent 0 - 255."""
        raise AttributeError("Output current is write-only")

    @output_current.setter
    def output_current(self, value: int) -> int:
        self._write_8(_REGISTER_IREFALL, value)

    @property
    def over_temp(self) -> bool:
        """True indicates over temperature condition."""
        return bool(self.read_register(_REGISTER_MODE2, mask = _1_BIT, offset = _BIT_POS_OVERTEMP))
    
    @over_temp.setter
    def over_temp(self) -> NoReturn:
        raise AttributeError("Over Temp is read-only")

    @property
    def errors_exist(self) -> bool:
        """True indicates errors exist."""
        return bool(self.read_register(_REGISTER_MODE2, mask = _1_BIT, offset = _BIT_POS_ERROR))
    
    @errors_exist.setter
    def errors_exist(self) -> NoReturn:
        raise AttributeError("Errors Exist is read-only")

    @property
    def low_power_mode(self) -> bool:
        return bool(self.read_register(_REGISTER_MODE1, mask = _1_BIT, offset = _BIT_POS_SLEEP))
    
    @low_power_mode.setter
    def low_power_mode(self, value: bool) -> None:
        self.write_register(_REGISTER_MODE1, value, mask = _1_BIT, offset =_BIT_POS_SLEEP)

    @property
    def aif(self) -> bool:
        return bool(self.read_register(_REGISTER_MODE1, mask = _1_BIT, offset = _BIT_POS_AIF))
    
    @aif.setter
    def aif(self, value: bool) -> NoReturn:
        raise AttributeError("AIF is read-only")

    @property
    def exponential_graduation(self) -> bool:
        """1 = exponential adjustment for gradation control, 0 = linear adjustment for gradation control (default)"""
        return bool(self.read_register(_REGISTER_MODE1, mask = _1_BIT, offset = _BIT_POS_EXP_EN))
    
    @exponential_graduation.setter
    def exponential_graduation(self, value: bool) -> None:
        self.write_register(_REGISTER_MODE1, int(value), mask = _1_BIT, offset =_BIT_POS_SLEEP)

    @property
    def group_blinking(self) -> bool:
        """1 = group control - blinking, 0 =  group control - dimming (default)"""
        return bool(self.read_register(_REGISTER_MODE1, mask = _1_BIT, offset = _BIT_POS_DMBLNK))
    
    @group_blinking.setter
    def group_blinking(self, value: bool) -> None:
        self.write_register(_REGISTER_MODE1, int(value), mask = _1_BIT, offset =_BIT_POS_DMBLNK)

    @property
    def group_pwm(self) -> int:
        """Global brightness control when group_blinking = 0)"""
        return self._read_8(_REGISTER_GRPPWM)
    
    @group_pwm.setter
    def group_pwm(self, value: int) -> None:
        self._write_8(_REGISTER_GRPPWM, value)

    @property
    def group_frequency(self) -> int:
        """Global blinking frequency  control when group_blinking = 1)"""
        return self._read_8(_REGISTER_GRPFREQ)
    
    @group_frequency.setter
    def group_frequency(self, value: int) -> None:
        self._write_8(_REGISTER_GRPFREQ, value)

    def clear_errors(self) -> None:
        self.write_register(_REGISTER_MODE2, 0x01, mask = _1_BIT, offset = _BIT_POS_CLRERR)

    SUBADR1 = const(0) # offset from _REGISTER_SUBADR1
    SUBADR2 = const(1)
    SUBADR3 = const(2)
    ALLCALLADR = const(3)

    def get_i2c_address(self, addr:int) -> int:
        if addr not in [PCA9955.SUBADR1, PCA9955.SUBADR2, PCA9955.SUBADR3, PCA9955.ALLCALLADR ]:
            raise ValueError(f"Value must be bone of {PCA9955.SUBADR1}, {PCA9955.SUBADR2}, {PCA9955.SUBADR3} or {PCA9955.ALLCALLADR}")
        return int(self.read_register(_REGISTER_SUBADR1, index = addr, mask = _7_BITS, offset = _BIT_POS_SUBADR))

    def set_i2c_address(self, addr:int, value: int) -> None:
        if addr not in [PCA9955.SUBADR1, PCA9955.SUBADR2, PCA9955.SUBADR3, PCA9955.ALLCALLADR ]:
            raise ValueError(f"Value must be bone of {PCA9955.SUBADR1}, {PCA9955.SUBADR2}, {PCA9955.SUBADR3} or {PCA9955.ALLCALLADR}")
        self.write_register(_REGISTER_SUBADR1, value, index = addr, mask = _7_BITS, offset = _BIT_POS_SUBADR)


    def read_register(self, base_register: int, index: int = 0, mask: int = 0xFF, offset: int = 0) -> int:
        """Read set of bits from register"""
        register = base_register + index
        mask = mask << offset
        reg = self._read_8(register)
        result = (reg & mask) >> offset
        print(f"rr - reg:{register:#x} offset:{offset:#x} reg: {reg:08b} mask:{mask:08b} value:0x{result:02x}")
        return result

    def write_register(self, base_register: int, value: int, index: int = 0, mask: int = 0xFF, offset: int = 0) -> None:
        """Write set of bits to register"""
        register = base_register + index
        mask = mask << offset
        inverse_mask = ~mask & 0xFF
        reg = self._read_8(register)
        value = (reg & inverse_mask) | (value << offset)
        self._write_8(register, value)
        print(f"wr - reg:{register:#x} offset:{offset:#x} reg: {reg:08b} mask:{mask:08b} value:0x{value:02x}")

    def read_channel_config(self, base_register: int, index: int) -> int:
        """Read channel configuration register"""
        register = base_register + (index >> 2)
        offset = (index % 4) << 1
        mask = _2_BITS << offset
        reg = self._read_8(register)
        result = (reg & mask) >> offset
        print(f"rc - reg:{register:#x} offset:{offset:#x} reg: {reg:08b} mask:{mask:08b} value:0x{result:02x}")
        return result

    def write_channel_config(self, base_register: int, index: int, value: int) -> None:
        """Write channel configuration register"""
        register = base_register + (index >> 2)
        offset = (index % 4) << 1
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
        return result[0]

    def _write_8(self, address: int, value: int) -> None:
        """ write a byte to the specified 8-bit register address."""
        result = bytearray(1)
        with self._device as i2c:
            i2c.write(bytes([address, value]))
            i2c.write(bytes([address]))
            i2c.readinto(result)
