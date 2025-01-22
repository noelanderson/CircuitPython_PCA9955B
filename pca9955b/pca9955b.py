# SPDX-FileCopyrightText: Copyright (c) 2024 Noel Anderson
#
# SPDX-License-Identifier: MIT
"""
`pca9955b`
================================================================================

CircuitPython helper library for the NXP 16-Channel I�C-Bus Constant-Current LED Driver


* Author(s): Noel Anderson

Implementation Notes
--------------------

**Hardware:**

* PCA9955 <https://www.nxp.com/docs/en/data-sheet/PCA9955B.pdf>

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads


"""

# imports

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/NoelAnderson/CircuitPython_PCA9955B.git"

import time

import microcontroller
from micropython import const

try:
    from types import TracebackType
    from typing import NoReturn, Optional, Type

    from busio import I2C
except ImportError:
    pass

from adafruit_bus_device import i2c_device
from digitalio import DigitalInOut, Direction, DriveMode
from micropython import const

from .pca9955b_groups import Groups

# Register map & bit positions for PCA9955B

_REG_MODE1 = const(0x00)  # R/W
# ---------------------------------------------------------------#
#    7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#   AIF  |  AI1  |  AI0  | SLEEP |  SUB1 |  SUB2 |  SUB3 |ALLCALL|
# ---------------------------------------------------------------#
_BIT_ALLCALL = const(0)  # R
_BIT_SUB3 = const(1)  # R/W
_BIT_SUB2 = const(2)  # R/W
_BIT_SUB1 = const(3)  # R/W
_BIT_SLEEP = const(4)  # R/W
_BIT_AI0 = const(5)  # R/W
_BIT_AI1 = const(6)  # R/W
_BIT_AIF = const(7)  # R/W

_REG_MODE2 = const(0x01)  # R/W
# ---------------------------------------------------------------#
#    7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#  OVERTP| ERROR | DMBLNK| CLRERR|  OCH  |EXP_EN |   -   |   -   |
# ---------------------------------------------------------------#
_BIT_EXP_EN = const(2)  # R/W
_BIT_OCH = const(3)  # R/W
_BIT_CLRERR = const(4)  # W
_BIT_DMBLNK = const(5)  # R/W
_BIT_ERROR = const(6)  # R
_BIT_OVERTEMP = const(7)  # R

_REG_LEDOUT0 = const(0x02)  # R/W
# ---------------------------------------------------------------#
#    7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#       LED3     |      LED2     |      LED1     |      LEDO     | LEDOUT0
# -------+-------+-------+-------+-------+-------+-------+-------|
#       LED7     |      LED6     |      LED5     |      LED4     | LEDOUT1
# -------+-------+-------+-------+-------+-------+-------+-------|
#       LED11    |      LED10    |      LED9     |      LED8     | LEDOUT2
# -------+-------+-------+-------+-------+-------+-------+-------|
#       LED15    |      LED14    |      LED13    |      LED12    | LEDOUT3
# ---------------------------------------------------------------#

_REG_GRPPWM = const(0x06)  # R/W
_REG_GRPFREQ = const(0x07)  # R/W

_REG_PWM0 = const(0x08)  # R/W  (PWM1 - PWM15 repeats)

_REG_IREF0 = const(0x18)  # R/W  (IREF1 - IREF15 repeats)

_REG_GRAD_MODE_SEL0 = const(0x38)  # R/W
_REG_GRAD_GRP_SEL0 = const(0x3A)  # R/W
# ---------------------------------------------------------------#
#    7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#       LED3     |      LED2     |      LED1     |      LEDO     | GRAD_GRP_SEL0
# -------+-------+-------+-------+-------+-------+-------+-------|
#       LED7     |      LED6     |      LED5     |      LED4     | GRAD_GRP_SEL1
# -------+-------+-------+-------+-------+-------+-------+-------|
#       LED11    |      LED10    |      LED9     |      LED8     | GRAD_GRP_SEL2
# -------+-------+-------+-------+-------+-------+-------+-------|
#       LED15    |      LED14    |      LED13    |      LED12    | GRAD_GRP_SEL3
# ---------------------------------------------------------------#

_REG_OFFSET = const(0x3F)  # R/W
# ---------------------------------------------------------------#
#    7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#                -               |        OUTPUT_DELAY           |
# ---------------------------------------------------------------#
_BIT_OUTPUT_DELAY = const(0)  # R/W

_REG_SUBADR1 = const(0x40)  # R/W
_REG_SUBADR2 = const(0x41)  # R/W
_REG_SUBADR3 = const(0x42)  # R/W
_REG_ALLCALLADR = const(0x43)  # R/W
# ---------------------------------------------------------------#
#    7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#                   I2C_BUS_SUBADDRESS                   |   -   |
# ---------------------------------------------------------------#
_BIT_SUBADR = const(1)  # R/W

_REG_PWMALL = const(0x44)  # R/W
_REG_IREFALL = const(0x45)  # R/W
_REG_EFLAG0 = const(0x46)  # R
# ---------------------------------------------------------------#
#    7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#       LED3     |      LED2     |      LED1     |      LEDO     | EFLAG0
# -------+-------+-------+-------+-------+-------+-------+-------|
#       LED7     |      LED6     |      LED5     |      LED4     | EFLAG1
# -------+-------+-------+-------+-------+-------+-------+-------|
#       LED11    |      LED10    |      LED9     |      LED8     | EFLAG2
# -------+-------+-------+-------+-------+-------+-------+-------|
#       LED15    |      LED14    |      LED13    |      LED12    | EFLAG3
# ---------------------------------------------------------------#


# Constants
_PCA9955B_DEFAULT_I2C_ADDR = const(0x3F)  # AD10 AD1 & AD2 all FLT (Floating Inputs)

# Bit Masks
_1_BIT = const(0b00000001)
_2_BITS = const(0b00000011)
_4_BITS = const(0b00001111)
_7_BITS = const(0b01111111)


class LedChannel:
    """
    A class representing a single PCA9955 channel.

    :param PCA9955 device: The PCA9955 device object.
    :param int index: The index of the channel.
    """

    E_NONE = const(0x00)
    """Constant for no error."""
    E_SHORT_CIRCUIT = const(0x01)
    """Constant for short circuit error."""
    E_OPEN_CIRCUIT = const(0x02)
    """Constant for open circuit error. """

    OFF = const(0x00)
    """Constant for channel off state."""
    ON = const(0x01)
    """Constant for channel full on state."""
    PWM = const(0x02)
    """Constant for channel PWM state."""
    PWM_GRP = const(0x03)
    """Constant for channel PWM group state."""

    def __init__(self, device: "PCA9955", index: int):
        self._device = device
        self._index = index

    @property
    def brightness(self) -> int:
        """Channel brightness 0 - 255."""
        return self._device.read_8(_REG_PWM0 + self._index)

    @brightness.setter
    def brightness(self, value: int) -> int:
        if not 0 <= value <= 255:
            raise ValueError("Value must be between 0 & 255")
        self._device.write_8(_REG_PWM0 + self._index, value)

    @property
    def gain(self) -> int:
        """Channel curent gain 0 - 255."""
        return self._device.read_8(_REG_IREF0 + self._index)

    @gain.setter
    def gain(self, value: int) -> int:
        if not 0 <= value <= 255:
            raise ValueError("Value must be between 0 & 255")
        self._device.write_8(_REG_IREF0 + self._index, value)

    @property
    def output_state(self) -> int:
        """Channel Driver output state"""
        return self._read_channel_config(_REG_LEDOUT0, self._index)

    @output_state.setter
    def output_state(self, value: int) -> int:
        if not LedChannel.OFF <= value <= LedChannel.PWM_GRP:
            raise ValueError(f"Value must be between {LedChannel.OFF} & {LedChannel.PWM_GRP}")
        self._write_channel_config(_REG_LEDOUT0, self._index, value)

    @property
    def led_error(self) -> int:
        """LED error state"""
        return self._read_channel_config(_REG_EFLAG0, self._index)

    @property
    def group(self) -> int:
        """Gradation group."""
        return self._read_channel_config(_REG_GRAD_GRP_SEL0, self._index)

    @group.setter
    def group(self, value: int) -> int:
        if not 0 <= value <= 3:
            raise ValueError("Group must be between 0 and 3")
        self._write_channel_config(_REG_GRAD_GRP_SEL0, self._index, value)

    @property
    def graduation_mode_select(self) -> bool:
        """1 = grad mode, 0 = normal"""
        offset = self._index % 8
        index = 0 if self._index < 7 else 1
        return bool(
            self._device.read_register(
                _REG_GRAD_MODE_SEL0,
                offset=index,
                mask=_1_BIT,
                bit_offset=offset,
            )
        )

    @graduation_mode_select.setter
    def graduation_mode_select(self, value: bool) -> None:
        offset = self._index % 8
        index = 0 if self._index < 7 else 1
        self._device.write_register(
            _REG_GRAD_MODE_SEL0,
            int(value),
            offset=index,
            mask=_1_BIT,
            bit_offset=offset,
        )

    def _read_channel_config(self, base_register: int, offset: int) -> int:
        """
        Reads the configuration of a specific channel from the device.

        :param int base_register: The base register address to read from.
        :param int offset: The offset to determine the specific channel configuration.

        :returns: The configuration value of the specified channel.
        :rtype: int

        This function calculates the appropriate register and mask to read the
        configuration of a specific channel. It then reads the register value,
        applies the mask, and shifts the result to obtain the channel configuration.
        """
        register = base_register + (offset >> 2)
        offset = (offset % 4) << 1
        mask = _2_BITS << offset
        reg = self._device.read_8(register)
        result = (reg & mask) >> offset
        # print(
        #     f"rc - reg:{register:#x} offset:{offset:#x} reg: {reg:08b}\
        #      mask:{mask:08b} value:0x{result:02x}"
        # )
        return result

    def _write_channel_config(self, base_register: int, offset: int, value: int) -> None:
        """
        Write channel configuration register.

        This method writes a value to a specific channel configuration register
        by calculating the appropriate register address and bit offset.

        :param int base_register: The base address of the register.
        :param int offset: The offset to determine the specific channel.
        :param int value: The value to write to the register.

        :returns: None
        """
        register = base_register + (offset >> 2)
        offset = (offset % 4) << 1
        mask = _2_BITS << offset
        inverse_mask = ~mask & 0xFF
        reg = self._device.read_8(register)
        value = (reg & inverse_mask) | (value << offset)
        self._device.write_8(register, value)
        # print(
        #     f"wc - reg:{register:#x} offset:{offset:#x} reg: {reg:08b}\
        #      mask:{mask:08b} value:0x{value:02x}"
        # )


class LedChannels:
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


class SubAddress:
    """A single PCA9955 I2C Sub Address.

    :param PCA9955 device: The PCA9955 device object
    :param int index: The index of the address
    """

    SUBADR1 = const(0)
    """Constant for SUBADR1."""
    SUBADR2 = const(1)
    """Constant for SUBADR2."""
    SUBADR3 = const(2)
    """Constant for SUBADR3."""
    ALLCALLADR = const(3)
    """Constant for ALLCALLADR."""

    def __init__(self, device: "PCA9955", index: int):
        self._device = device
        self._index = index

    @property
    def address(self) -> int:
        """Bus Address for the I2C Sub Address."""
        return int(
            self._device.read_register(
                _REG_SUBADR1,
                offset=self._index,
                mask=_7_BITS,
                bit_offset=_BIT_SUBADR,
            )
        )

    @address.setter
    def address(self, value: int) -> int:
        """Set the Bus address of the I2C Sub Address."""
        self._device.write_register(
            _REG_SUBADR1,
            value,
            offset=self._index,
            mask=_7_BITS,
            bit_offset=_BIT_SUBADR,
        )

    @property
    def enable(self) -> bool:
        """I2C sub address enabled."""
        bit_offset = SubAddress.ALLCALLADR - self._index  # Bits in reverse order to registers
        return bool(self._device.read_register(_REG_MODE1, mask=_1_BIT, bit_offset=bit_offset))

    @enable.setter
    def enable(self, value: bool) -> None:
        """Enable I2C sub address."""
        bit_offset = SubAddress.ALLCALLADR - self._index  # Bits in reverse order to registers
        self._device.write_register(_REG_MODE1, int(value), mask=_1_BIT, bit_offset=bit_offset)


class SubAddresses:
    """Lazily creates and caches I2CSubAddress objects as needed. Treat it like a sequence.

    :param PCA9955 device: The PCA9955 device object
    """

    def __init__(self, device: "PCA9955") -> None:
        self._device = device
        self.i2c_addresses = [None] * len(self)

    def __len__(self) -> int:
        return 4

    def __getitem__(self, index: int) -> SubAddress:
        if not self.i2c_addresses[index]:
            self.i2c_addresses[index] = SubAddress(self._device, index)
        return self.i2c_addresses[index]


class PCA9955:  # noqa PLR0904
    """
    A class representing a PCA9955 device.

    This class provides an interface to control the PCA9955 LED driver via I2C.
    It allows for setting various properties such as brightness, gain, and
    different modes of operation.
    It also provides methods to read and write to the device's registers.

    Attributes:
        channels (LedChannels): An collection of LedChannels objects.
        groups (Groups): A collection of Group Objects.
        subaddresses (I2CSubAddresses): A collection of I2CSubAddress objects.
    """

    def __init__(
        self,
        i2c: I2C,
        address: int = _PCA9955B_DEFAULT_I2C_ADDR,
        oe_pin: Optional[microcontroller.Pin] = None,
        reset_pin: Optional[microcontroller.Pin] = None,
    ) -> None:
        """
        Initialize the PCA9955B device at ``address`` on ``i2c_bus``
        If oe and/or reset pins are connected to GPIO outputs, as per the nxp application note
        then these can be passed in as ``oe_pin`` and ``reset_pin`` respectively and will be
        used by the reset and output_enable properties.

        :param i2c: The I2C bus the device is connected to.
        :type i2c: I2C
        :param address: The I2C address of the device. Default is _PCA9955B_DEFAULT_I2C_ADDR (0x3F).
        :type address: int, optional
        :param oe_pin: GPIO pin connected to the Output Enable (OE) pin of the device.
        :type oe_pin: Optional[microcontroller.Pin], optional
        :param reset_pin: GPIO pin connected to the Reset pin of the device. Default is None.
        :type reset_pin: Optional[microcontroller.Pin], optional

        :return: None
        """
        self._device = i2c_device.I2CDevice(i2c, address)
        self.channels = LedChannels(self)
        self.groups = Groups(self)
        self.subaddresses = SubAddresses(self)
        self._oe = None
        if oe_pin is not None:
            self._oe = DigitalInOut(oe_pin)
            self._oe.direction = Direction.OUTPUT
            self._oe.drive_mode = DriveMode.OPEN_DRAIN
        self._reset = None
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
        """If the Reset pin is tied to a GPIO then reset the PCA9955."""
        if self._reset is not None:
            self._reset.value = False
            time.sleep(0.01)
            self._reset.value = True

    @property
    def output_enable(self) -> Optional[bool]:
        """If the oe pin is tied to a GPIO then control the PCA9955 oe (output enable)."""
        if self._oe is not None:
            return not self._oe.value
        return None

    @output_enable.setter
    def output_enable(self, value: bool) -> None:
        if self._oe is not None:
            self._oe.value = not value

    @property
    def brightness(self) -> NoReturn:
        """Global brightness 0 - 255."""
        raise AttributeError("brightness is write-only")

    @brightness.setter
    def brightness(self, value: int) -> None:
        self.write_8(_REG_PWMALL, value)

    @property
    def gain(self) -> NoReturn:
        """Global output currrent 0 - 255."""
        raise AttributeError("Output current is write-only")

    @gain.setter
    def gain(self, value: int) -> int:
        self.write_8(_REG_IREFALL, value)

    @property
    def over_temp(self) -> bool:
        """True indicates over temperature condition."""
        return bool(self.read_register(_REG_MODE2, mask=_1_BIT, bit_offset=_BIT_OVERTEMP))

    @over_temp.setter
    def over_temp(self) -> NoReturn:  # noqa PLR6301
        raise AttributeError("Over Temp is read-only")

    @property
    def errors_exist(self) -> bool:
        """True indicates errors exist."""
        return bool(self.read_register(_REG_MODE2, mask=_1_BIT, bit_offset=_BIT_ERROR))

    @errors_exist.setter
    def errors_exist(self) -> NoReturn:  # noqa PLR6301
        raise AttributeError("Errors Exist is read-only")

    @property
    def low_power_mode(self) -> bool:
        """1 = low power mode, 0 = normal mode"""
        return bool(self.read_register(_REG_MODE1, mask=_1_BIT, bit_offset=_BIT_SLEEP))

    @low_power_mode.setter
    def low_power_mode(self, value: bool) -> None:
        self.write_register(_REG_MODE1, value, mask=_1_BIT, bit_offset=_BIT_SLEEP)

    @property
    def auto_increment_flag(self) -> bool:
        """1 = auto increment, 0 = normal mode"""
        return bool(self.read_register(_REG_MODE1, mask=_1_BIT, bit_offset=_BIT_AIF))

    @auto_increment_flag.setter
    def auto_increment_flag(self, value: bool) -> NoReturn:  # noqa PLR6301
        raise AttributeError("AIF is read-only")

    @property
    def auto_increment_mode(self) -> int:
        """Auto increment mode 0 - 3"""
        return self.read_register(_REG_MODE1, mask=_2_BITS, bit_offset=_BIT_AI0)

    @auto_increment_mode.setter
    def auto_increment_mode(self, value: int) -> None:
        if not 0 <= value <= 3:
            raise ValueError("Valid values are 0 - 3")
        self.write_register(_REG_MODE1, value, mask=_2_BITS, bit_offset=_BIT_AI0)

    @property
    def exponential_graduation(self) -> bool:
        """1 = exponential adjustment for gradation control,
        0 = linear adjustment for gradation control (default)"""
        return bool(self.read_register(_REG_MODE2, mask=_1_BIT, bit_offset=_BIT_EXP_EN))

    @exponential_graduation.setter
    def exponential_graduation(self, value: bool) -> None:
        self.write_register(_REG_MODE2, int(value), mask=_1_BIT, bit_offset=_BIT_EXP_EN)

    @property
    def group_blinking(self) -> bool:
        """1 = group control - blinking,
        0 =  group control - dimming (default)"""
        return bool(self.read_register(_REG_MODE2, mask=_1_BIT, bit_offset=_BIT_DMBLNK))

    @group_blinking.setter
    def group_blinking(self, value: bool) -> None:
        self.write_register(_REG_MODE2, int(value), mask=_1_BIT, bit_offset=_BIT_DMBLNK)

    @property
    def group_pwm(self) -> int:
        """Global brightness control when group_blinking = 0)"""
        return self.read_8(_REG_GRPPWM)

    @group_pwm.setter
    def group_pwm(self, value: int) -> None:
        self.write_8(_REG_GRPPWM, value)

    @property
    def group_frequency(self) -> int:
        """Global blinking frequency control when group_blinking = 1)"""
        return self.read_8(_REG_GRPFREQ)

    @group_frequency.setter
    def group_frequency(self, value: int) -> None:
        """Global blinking frequency control when group_blinking = 1)"""
        self.write_8(_REG_GRPFREQ, value)

    def clear_errors(self) -> None:
        """Clear errors"""
        self.write_register(_REG_MODE2, 0x01, mask=_1_BIT, bit_offset=_BIT_CLRERR)

    @property
    def output_delay_offset(self) -> int:
        """Output delay offset"""
        return self.read_register(_REG_OFFSET, mask=_4_BITS, bit_offset=_BIT_OUTPUT_DELAY)

    @output_delay_offset.setter
    def output_delay_offset(self, value: int) -> None:
        if value not in {0x00, 0x01, 0x02, 0x03, 0x0F}:
            raise ValueError(
                f"Value must be one of {0x00:00x}, {0x01:02x}, {0x02:02x}, {0x03:02x} or{0x0F:02x}"
            )
        self.write_register(
            _REG_OFFSET,
            value,
            mask=_4_BITS,
            bit_offset=_BIT_OUTPUT_DELAY,
        )

    def read_register(
        self,
        base_register: int,
        offset: int = 0,
        mask: int = 0xFF,
        bit_offset: int = 0,
    ) -> int:
        """
        Read a set of bits from a register.

        :param int base_register: The base register address.
        :param int offset: The offset to add to the base register. Defaults to 0.
        :param int mask: The bitmask to apply to the register value. Defaults to 0xFF.
        :param int bit_offset: The bit offset to apply to the mask. Defaults to 0.

        :returns: The value read from the register after applying the mask and bit offset.
        :rtype: int
        :meta private:
        """
        register = base_register + offset
        mask = mask << bit_offset
        reg = self.read_8(register)
        result = (reg & mask) >> bit_offset
        # print(
        #     f"rr - reg:{register:#x} offset:{bit_offset:#x} reg: {reg:08b}\
        #      mask:{mask:08b} value:0x{result:02x}"
        # )
        return result

    def write_register(  # noqa: PLR0913
        self,
        base_register: int,
        value: int,
        offset: int = 0,
        mask: int = 0xFF,
        bit_offset: int = 0,
    ) -> None:
        """
        Write a set of bits to a register.

        :param int base_register: The base address of the register.
        :param int value: The value to write to the register.
        :param int offset: The offset to add to the base register. Defaults to 0.
        :param int mask: The mask to apply to the value. Defaults to 0xFF.
        :param int bit_offset: The bit offset to apply to the value. Defaults to 0.

        :returns: None
        :meta private:
        """
        register = base_register + offset
        mask = mask << bit_offset
        inverse_mask = ~mask & 0xFF
        reg = self.read_8(register)
        value = (reg & inverse_mask) | (value << bit_offset)
        self.write_8(register, value)
        # print(
        #     f"wr - reg:{register:#x} offset:{bit_offset:#x} reg: {reg:08b}\
        #      mask:{mask:08b} value:0x{value:02x}"
        # )

    def read_8(self, address: int) -> int:
        """
        Read and return a byte from the specified 8-bit register address.

        :meta private:
        """
        result = bytearray(1)
        with self._device as i2c:
            i2c.write(bytes([address]))
            i2c.readinto(result)
        # print(f"r8 - address:{address:#x} value:{result[0]:08b}")
        return result[0]

    def write_8(self, address: int, value: int) -> None:
        """
        Write a byte to the specified 8-bit register address.

        :meta private:
        """
        result = bytearray(1)
        with self._device as i2c:
            i2c.write(bytes([address, value]))
            i2c.write(bytes([address]))
            i2c.readinto(result)
        # print(f"w8 - address:{address:#x} value:{value:08b}")
