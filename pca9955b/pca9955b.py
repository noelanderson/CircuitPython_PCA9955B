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
    from typing import Optional, NoReturn, Type
    from types import TracebackType
    from busio import I2C
except ImportError:
    pass

from adafruit_bus_device import i2c_device
from digitalio import DigitalInOut, Direction, DriveMode
import pca9955_registers as PCA9955REG


# Constants
_PCA9955B_DEFAULT_I2C_ADDR = const(0x3F)  # AD10 AD1 & AD2 all FLT (Floating Inputs)

# Bit Masks
_1_BIT = const(0b00000001)
_2_BITS = const(0b00000011)
_3_BITS = const(0b00000111)
_4_BITS = const(0b00001111)
_6_BITS = const(0b00111111)
_7_BITS = const(0b01111111)


class LedChannel:
    """A class representing a single PCA9955 channel.

    :param PCA9955 device: The PCA9955 device object.
    :param int index: The index of the channel.

    Attributes:
        NONE (int): Constant for no error.
        SHORT_CIRCUIT (int): Constant for short circuit error.
        OPEN_CIRCUIT (int): Constant for open circuit error.
        OFF (int): Constant for channel off state.
        FULL_ON (int): Constant for channel full on state.
        PWM (int): Constant for channel PWM state.
        PWM_GRP (int): Constant for channel PWM group state.
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
        return self._device.read_8(PCA9955REG.PWM0 + self._index)

    @brightness.setter
    def brightness(self, value: int) -> int:
        if not 0 <= value <= 255:
            raise ValueError("Value must be between 0 & 255")
        self._device.write_8(PCA9955REG.PWM0 + self._index, value)

    @property
    def gain(self) -> int:
        """Channel curent gain 0 - 255."""
        return self._device.read_8(PCA9955REG.IREF0 + self._index)

    @gain.setter
    def gain(self, value: int) -> int:
        if not 0 <= value <= 255:
            raise ValueError("Value must be between 0 & 255")
        self._device.write_8(PCA9955REG.IREF0 + self._index, value)

    @property
    def output_state(self) -> int:
        """Channel Driver output state"""

        return self._read_channel_config(PCA9955REG.LEDOUT0, self._index)

    @output_state.setter
    def output_state(self, value: int) -> int:
        if not LedChannel.OFF <= value <= LedChannel.PWM_GRP:
            raise ValueError(
                f"Value must be between {LedChannel.OFF} & {LedChannel.PWM_GRP}"
            )
        self._write_channel_config(PCA9955REG.LEDOUT0, self._index, value)

    @property
    def led_error(self) -> int:
        """LED error state"""
        return self._device.read_channel_config(PCA9955REG.EFLAG0, self._index)

    @property
    def group(self) -> int:
        """Gradation group."""
        return self._read_channel_config(PCA9955REG.GRAD_GRP_SEL0, self._index)

    @group.setter
    def group(self, value: int) -> int:
        if not 0 <= value <= 3:
            raise ValueError("Group must be between 0 and 3")
        self._write_channel_config(PCA9955REG.GRAD_GRP_SEL0, self._index, value)

    @property
    def graduation_mode_select(self) -> bool:
        """1 = grad mode, 0 = normal"""
        offset = self._index % 8
        index = 0 if self._index < 7 else 1
        return bool(
            self._device.read_register(
                PCA9955REG.GRAD_MODE_SEL0,
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
            PCA9955REG.GRAD_MODE_SEL0,
            int(value),
            offset=index,
            mask=_1_BIT,
            bit_offset=offset,
        )

    def _read_channel_config(self, base_register: int, offset: int) -> int:
        """
        Reads the configuration of a specific channel from the device.

        Args:
            base_register (int): The base register address to read from.
            offset (int): The offset to determine the specific channel configuration.

        Returns:
            int: The configuration value of the specified channel.

        This function calculates the appropriate register and mask to read the
        configuration of a specific channel. It then reads the register value,
        applies the mask, and shifts the result to obtain the channel configuration.
        """
        register = base_register + (offset >> 2)
        offset = (offset % 4) << 1
        mask = _2_BITS << offset
        reg = self._device.read_8(register)
        result = (reg & mask) >> offset
        print(
            f"rc - reg:{register:#x} offset:{offset:#x} reg: {reg:08b}\
             mask:{mask:08b} value:0x{result:02x}"
        )
        return result

    def _write_channel_config(
        self, base_register: int, offset: int, value: int
    ) -> None:
        """Write channel configuration register.

        This method writes a value to a specific channel configuration register
        by calculating the appropriate register address and bit offset.

        Args:
            base_register (int): The base address of the register.
            offset (int): The offset to determine the specific channel.
            value (int): The value to write to the register.

        Returns:
            None
        """
        register = base_register + (offset >> 2)
        offset = (offset % 4) << 1
        mask = _2_BITS << offset
        inverse_mask = ~mask & 0xFF
        reg = self._device.read_8(register)
        value = (reg & inverse_mask) | (value << offset)
        self._device.write_8(register, value)
        print(
            f"wc - reg:{register:#x} offset:{offset:#x} reg: {reg:08b}\
             mask:{mask:08b} value:0x{value:02x}"
        )


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
    """A single PCA9955 Graduation Group.

    :param PCA9955 device: The PCA9955 device object
    :param int index: The index of the group
    """

    def __init__(self, device: "PCA9955", index: int):
        self._device = device
        self._index = index

    @property
    def ramp_up(self) -> bool:
        """Ramp-up enable/disable."""
        return self._device.read_register(
            PCA9955REG.RAMP_RATE_GRP0,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG.BIT_RAMP_UP_ENABLE,
        )

    @ramp_up.setter
    def ramp_up(self, value: bool) -> bool:
        self._device.write_register(
            PCA9955REG.RAMP_RATE_GRP0,
            value,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG.BIT_RAMP_UP_ENABLE,
        )

    @property
    def ramp_down(self) -> bool:
        """Ramp-down enable/disable."""
        return self._device.read_register(
            PCA9955REG.RAMP_RATE_GRP0,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG.BIT_RAMP_DOWN_ENABLE,
        )

    @ramp_down.setter
    def ramp_down(self, value: bool) -> bool:
        self._device.write_register(
            PCA9955REG.RAMP_RATE_GRP0,
            value,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG.BIT_RAMP_DOWN_ENABLE,
        )

    @property
    def ramp_rate(self) -> int:
        """Ramp rate per step 0 - 64."""
        return self._device.read_register(
            PCA9955REG.RAMP_RATE_GRP0,
            offset=self._index,
            mask=_6_BITS,
            bit_offset=PCA9955REG.BIT_RAMP_RATE,
        )

    @ramp_rate.setter
    def ramp_rate(self, value: int) -> int:
        if not 0 <= value <= 64:
            raise ValueError("Value must be between 0 & 64")
        self._device.write_register(
            PCA9955REG.RAMP_RATE_GRP0,
            value,
            offset=self._index,
            mask=_6_BITS,
            bit_offset=PCA9955REG.BIT_RAMP_RATE,
        )

    @property
    def cycle_time(self) -> int:
        """Cycle time - 0 (0.5ms) or 1 (8ms)."""
        return self._device.read_register(
            PCA9955REG.STEP_TIME_GRP0,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG.BIT_CYCLE_TIME,
        )

    @cycle_time.setter
    def cycle_time(self, value: int) -> int:
        if not 0 <= value <= 1:
            raise ValueError("Valid values are 0 (0.5ms) or 1 (8ms)")
        self._device.write_register(
            PCA9955REG.STEP_TIME_GRP0,
            value,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG.BIT_CYCLE_TIME,
        )

    @property
    def factor_per_step(self) -> int:
        """Multiple factor per step 0 - 64."""
        return self._device.read_register(
            PCA9955REG.STEP_TIME_GRP0,
            offset=self._index,
            mask=_6_BITS,
            bit_offset=PCA9955REG.BIT_FACTOR_PER_STEP,
        )

    @factor_per_step.setter
    def factor_per_step(self, value: int) -> int:
        if not 0 <= value <= 64:
            raise ValueError("Value must be between 0 & 64")
        self._device.write_register(
            PCA9955REG.STEP_TIME_GRP0,
            value,
            offset=self._index,
            mask=_6_BITS,
            bit_offset=PCA9955REG.BIT_FACTOR_PER_STEP,
        )

    @property
    def hold_on(self) -> bool:
        """Hold on enable/disable."""
        return self._device.read_register(
            PCA9955REG.HOLD_CNTL_GRP0,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG.BIT_HOLD_ON_ENABLE,
        )

    @hold_on.setter
    def hold_on(self, value: bool) -> bool:
        self._device.write_register(
            PCA9955REG.HOLD_CNTL_GRP0,
            value,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG.BIT_HOLD_ON_ENABLE,
        )

    @property
    def hold_off(self) -> bool:
        """Hold off enable/disable."""
        return self._device.read_register(
            PCA9955REG.HOLD_CNTL_GRP0,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG.BIT_HOLD_OFF_ENABLE,
        )

    @hold_off.setter
    def hold_off(self, value: bool) -> bool:
        self._device.write_register(
            PCA9955REG.HOLD_CNTL_GRP0,
            value,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG.BIT_HOLD_OFF_ENABLE,
        )

    @property
    def hold_on_time(self) -> int:
        """Hold On time - 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s)
        , 4 (1s), 5 (2s), 6 (4s), 7 (6s)."""
        return self._device.read_register(
            PCA9955REG.HOLD_CNTL_GRP0,
            offset=self._index,
            mask=_3_BITS,
            bit_offset=PCA9955REG.BIT_HOLD_ON_TIME,
        )

    @hold_on_time.setter
    def hold_on_time(self, value: int) -> int:
        if not 0 <= value <= 7:
            raise ValueError(
                "Valid values are 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s)\
                , 4 (1s), 5 (2s), 6 (4s), 7 (6s)"
            )
        self._device.write_register(
            PCA9955REG.HOLD_CNTL_GRP0,
            value,
            offset=self._index,
            mask=_3_BITS,
            bit_offset=PCA9955REG.BIT_HOLD_ON_TIME,
        )

    @property
    def hold_off_time(self) -> int:
        """Hold On time  - 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s)\
        , 4 (1s), 5 (2s), 6 (4s), 7 (6s)."""
        return self._device.read_register(
            PCA9955REG.HOLD_CNTL_GRP0,
            offset=self._index,
            mask=_3_BITS,
            bit_offset=PCA9955REG.BIT_HOLD_OFF_TIME,
        )

    @hold_off_time.setter
    def hold_off_time(self, value: int) -> int:
        if not 0 <= value <= 7:
            raise ValueError(
                "Valid values are 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s)\
                , 4 (1s), 5 (2s), 6 (4s), 7 (6s)"
            )
        self._device.write_register(
            PCA9955REG.HOLD_CNTL_GRP0,
            value,
            offset=self._index,
            mask=_3_BITS,
            bit_offset=PCA9955REG.BIT_HOLD_OFF_TIME,
        )

    @property
    def gain(self) -> int:
        """Group output current gain (0-255)."""
        return self._device.read_register(PCA9955REG.IREF_GRP0, self._index)

    @gain.setter
    def gain(self, value: int) -> int:
        """Set group output current gain to specified value. Should be 0-255."""
        if not 0 <= value <= 255:
            raise ValueError("Valid values are  0-255")
        self._device.write_register(PCA9955REG.IREF_GRP0, value, offset=self._index)

    @property
    def graduation_mode(self) -> bool:
        """Graduation mode. 0 = Single Shot, 1 = Continuous."""
        bit_offset = self._index << 2
        return bool(
            self._device.read_register(
                PCA9955REG.GRAD_CNTL, mask=_1_BIT, bit_offset=bit_offset
            )
        )

    @graduation_mode.setter
    def graduation_mode(self, value: bool) -> None:
        """Set graduation mode. 0 = Single Shot, 1 = Continuous."""
        bit_offset = self._index << 2
        self._device.write_register(
            PCA9955REG.GRAD_CNTL, int(value), mask=_1_BIT, bit_offset=bit_offset
        )

    def graduation_start(self) -> None:
        """Start defined Group Graduation running."""
        bit_offset = (self._index << 2) + 1
        self._device.write_register(
            PCA9955REG.GRAD_CNTL, 0x01, mask=_1_BIT, bit_offset=bit_offset
        )

    def graduation_stop(self) -> None:
        """Stop Group Graduation."""
        bit_offset = (self._index << 2) + 1
        self._device.write_register(
            PCA9955REG.GRAD_CNTL, 0x00, mask=_1_BIT, bit_offset=bit_offset
        )


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


class I2CSubAddress:  # pylint: disable=too-few-public-methods
    """A single PCA9955 I2C Sub Address.

    :param PCA9955 device: The PCA9955 device object
    :param int index: The index of the address
    """

    # I2C Sub Address constants
    SUBADR1 = const(0)  # offset from PCA9955REG.SUBADR1
    SUBADR2 = const(1)
    SUBADR3 = const(2)
    ALLCALLADR = const(3)

    def __init__(self, device: "PCA9955", index: int):
        self._device = device
        self._index = index

    @property
    def address(self) -> int:
        """Bus Address of the I2C Sub Address."""
        return int(
            self._device.read_register(
                PCA9955REG.SUBADR1,
                offset=self._index,
                mask=_7_BITS,
                bit_offset=PCA9955REG.BIT_SUBADR,
            )
        )

    @address.setter
    def address(self, value: int) -> int:
        """Set the Bus address of the I2C Sub Address."""
        self._device.write_register(
            PCA9955REG.SUBADR1,
            value,
            offset=self._index,
            mask=_7_BITS,
            bit_offset=PCA9955REG.BIT_SUBADR,
        )

    @property
    def enable(self) -> bool:
        """I2C sub address enabled."""
        bit_offset = (
            I2CSubAddress.ALLCALLADR - self._index
        )  # Bits in reverse order to registers
        return bool(
            self._device.read_register(
                PCA9955REG.MODE1, mask=_1_BIT, bit_offset=bit_offset
            )
        )

    @enable.setter
    def enable(self, value: bool) -> None:
        """Enable I2C sub address."""
        bit_offset = (
            I2CSubAddress.ALLCALLADR - self._index
        )  # Bits in reverse order to registers
        self._device.write_register(
            PCA9955REG.MODE1, int(value), mask=_1_BIT, bit_offset=bit_offset
        )


class I2CSubAddresses:  # pylint: disable=too-few-public-methods
    """Lazily creates and caches I2CSubAddress objects as needed. Treat it like a sequence.

    :param PCA9955 device: The PCA9955 device object
    """

    def __init__(self, device: "PCA9955") -> None:
        self._device = device
        self.i2c_addresses = [None] * len(self)

    def __len__(self) -> int:
        return 4

    def __getitem__(self, index: int) -> I2CSubAddress:
        if not self.i2c_addresses[index]:
            self.i2c_addresses[index] = I2CSubAddress(self._device, index)
        return self.i2c_addresses[index]


class PCA9955:
    """A class representing a PCA9955 device.

    This class provides an interface to control the PCA9955 LED driver via I2C.
    It allows for setting various properties such as brightness, gain, and
    different modes of operation.
    It also provides methods to read and write to the device's registers.

    Attributes:
        channels (LedChannels): An instance of the LedChannels class to control
        individual LED channels.
        groups (Groups): An instance of the Groups class to control groups of LEDs.
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
        self.subaddresses = I2CSubAddresses(self)
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
        self.write_8(PCA9955REG.PWMALL, value)

    @property
    def gain(self) -> NoReturn:
        """Global output currrent 0 - 255."""
        raise AttributeError("Output current is write-only")

    @gain.setter
    def gain(self, value: int) -> int:
        self.write_8(PCA9955REG.IREFALL, value)

    @property
    def over_temp(self) -> bool:
        """True indicates over temperature condition."""
        return bool(
            self.read_register(
                PCA9955REG.MODE2, mask=_1_BIT, bit_offset=PCA9955REG.BIT_OVERTEMP
            )
        )

    @over_temp.setter
    # pylint: disable=no-self-use
    def over_temp(self) -> NoReturn:
        raise AttributeError("Over Temp is read-only")

    @property
    def errors_exist(self) -> bool:
        """True indicates errors exist."""
        return bool(
            self.read_register(
                PCA9955REG.MODE2, mask=_1_BIT, bit_offset=PCA9955REG.BIT_ERROR
            )
        )

    @errors_exist.setter
    # pylint: disable=no-self-use
    def errors_exist(self) -> NoReturn:
        raise AttributeError("Errors Exist is read-only")

    @property
    def low_power_mode(self) -> bool:
        """1 = low power mode, 0 = normal mode"""
        return bool(
            self.read_register(
                PCA9955REG.MODE1, mask=_1_BIT, bit_offset=PCA9955REG.BIT_SLEEP
            )
        )

    @low_power_mode.setter
    def low_power_mode(self, value: bool) -> None:
        self.write_register(
            PCA9955REG.MODE1, value, mask=_1_BIT, bit_offset=PCA9955REG.BIT_SLEEP
        )

    @property
    def auto_increment_flag(self) -> bool:
        """1 = auto increment, 0 = normal mode"""
        return bool(
            self.read_register(
                PCA9955REG.MODE1, mask=_1_BIT, bit_offset=PCA9955REG.BIT_AIF
            )
        )

    @auto_increment_flag.setter
    # pylint: disable=no-self-use
    def auto_increment_flag(self, value: bool) -> NoReturn:
        raise AttributeError("AIF is read-only")

    @property
    def auto_increment_mode(self) -> int:
        """Auto increment mode 0 - 3"""
        return self.read_register(
            PCA9955REG.MODE1, mask=_2_BITS, bit_offset=PCA9955REG.BIT_AI0
        )

    @auto_increment_mode.setter
    def auto_increment_mode(self, value: int) -> None:
        if not 0 <= value <= 3:
            raise ValueError("Valid values are 0 - 3")
        self.write_register(
            PCA9955REG.MODE1, value, mask=_2_BITS, bit_offset=PCA9955REG.BIT_AI0
        )

    @property
    def exponential_graduation(self) -> bool:
        """1 = exponential adjustment for gradation control,
        0 = linear adjustment for gradation control (default)"""
        return bool(
            self.read_register(
                PCA9955REG.MODE2, mask=_1_BIT, bit_offset=PCA9955REG.BIT_EXP_EN
            )
        )

    @exponential_graduation.setter
    def exponential_graduation(self, value: bool) -> None:
        self.write_register(
            PCA9955REG.MODE2, int(value), mask=_1_BIT, bit_offset=PCA9955REG.BIT_EXP_EN
        )

    @property
    def group_blinking(self) -> bool:
        """1 = group control - blinking,
        0 =  group control - dimming (default)"""
        return bool(
            self.read_register(
                PCA9955REG.MODE2, mask=_1_BIT, bit_offset=PCA9955REG.BIT_DMBLNK
            )
        )

    @group_blinking.setter
    def group_blinking(self, value: bool) -> None:
        self.write_register(
            PCA9955REG.MODE2, int(value), mask=_1_BIT, bit_offset=PCA9955REG.BIT_DMBLNK
        )

    @property
    def group_pwm(self) -> int:
        """Global brightness control when group_blinking = 0)"""
        return self.read_8(PCA9955REG.GRPPWM)

    @group_pwm.setter
    def group_pwm(self, value: int) -> None:
        self.write_8(PCA9955REG.GRPPWM, value)

    @property
    def group_frequency(self) -> int:
        """Global blinking frequency control when group_blinking = 1)"""
        return self.read_8(PCA9955REG.GRPFREQ)

    @group_frequency.setter
    def group_frequency(self, value: int) -> None:
        """Global blinking frequency control when group_blinking = 1)"""
        self.write_8(PCA9955REG.GRPFREQ, value)

    def clear_errors(self) -> None:
        """Clear errors"""
        self.write_register(
            PCA9955REG.MODE2, 0x01, mask=_1_BIT, bit_offset=PCA9955REG.BIT_CLRERR
        )

    @property
    def output_delay_offset(self) -> int:
        """Output delay offset"""
        return self.read_register(
            PCA9955REG.OFFSET, mask=_4_BITS, bit_offset=PCA9955REG.BIT_OUTPUT_DELAY
        )

    @output_delay_offset.setter
    def output_delay_offset(self, value: int) -> None:
        if value not in [0x00, 0x01, 0x02, 0x03, 0x0F]:
            raise ValueError(
                f"Value must be one of {0x00:00x}, {0x01:02x}, {0x02:02x}, {0x03:02x} or{0x0F:02x}"
            )
        self.write_register(
            PCA9955REG.OFFSET,
            value,
            mask=_4_BITS,
            bit_offset=PCA9955REG.BIT_OUTPUT_DELAY,
        )

    def read_register(
        self,
        base_register: int,
        offset: int = 0,
        mask: int = 0xFF,
        bit_offset: int = 0,
    ) -> int:
        """Read a set of bits from a register.

        Args:
            base_register (int): The base register address.
            offset (int, optional): The offset to add to the base register. Defaults to 0.
            mask (int, optional): The bitmask to apply to the register value. Defaults to 0xFF.
            bit_offset (int, optional): The bit offset to apply to the mask. Defaults to 0.

        Returns:
            int: The value read from the register after applying the mask and bit offset.
        """
        register = base_register + offset
        mask = mask << bit_offset
        reg = self.read_8(register)
        result = (reg & mask) >> bit_offset
        print(
            f"rr - reg:{register:#x} offset:{bit_offset:#x} reg: {reg:08b}\
             mask:{mask:08b} value:0x{result:02x}"
        )
        return result

    # pylint: disable= too-many-arguments
    def write_register(
        self,
        base_register: int,
        value: int,
        offset: int = 0,
        mask: int = 0xFF,
        bit_offset: int = 0,
    ) -> None:
        """Write a set of bits to a register.

        Args:
            base_register (int): The base address of the register.
            value (int): The value to write to the register.
            offset (int, optional): The offset to add to the base register.Defaults to 0.
            mask (int, optional): The mask to apply to the value. Defaults to 0xFF.
            bit_offset (int, optional): The bit offset to apply to the value. Defaults to 0.

        Returns:
            None
        """
        register = base_register + offset
        mask = mask << bit_offset
        inverse_mask = ~mask & 0xFF
        reg = self.read_8(register)
        value = (reg & inverse_mask) | (value << bit_offset)
        self.write_8(register, value)
        print(
            f"wr - reg:{register:#x} offset:{bit_offset:#x} reg: {reg:08b}\
             mask:{mask:08b} value:0x{value:02x}"
        )

    def read_8(self, address: int) -> int:
        """Read and return a byte from the specified 8-bit register address."""
        result = bytearray(1)
        with self._device as i2c:
            i2c.write(bytes([address]))
            i2c.readinto(result)
        print(f"r8 - address:{address:#x} value:{result[0]:08b}")
        return result[0]

    def write_8(self, address: int, value: int) -> None:
        """Write a byte to the specified 8-bit register address."""
        result = bytearray(1)
        with self._device as i2c:
            i2c.write(bytes([address, value]))
            i2c.write(bytes([address]))
            i2c.readinto(result)
        print(f"w8 - address:{address:#x} value:{value:08b}")
