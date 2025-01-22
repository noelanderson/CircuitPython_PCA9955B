# SPDX-FileCopyrightText: Copyright (c) 2024 Noel Anderson
#
# SPDX-License-Identifier: MIT
"""
`pca9955b_groups`
================================================================================

CircuitPython helper library for Group Control of the
NXP 16-Channel I�C-Bus Constant-Current LED Driver


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

from micropython import const

from . import pca9955b as PCA9955

# Register map & bit positions for Gropup functions of PCA9955B

PCA9955REG_RAMP_RATE_GRP0 = const(0x28)  # R/W
# ---------------------------------------------------------------#
#    7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#  RAMPUP| RAMPDW|              RAMP RATE                        | (GRP1 - GRP3 repeats)
# ---------------------------------------------------------------#
PCA9955REG_BIT_RAMP_RATE = const(0)  # R/W
PCA9955REG_BIT_RAMP_DOWN_ENABLE = const(6)  # R/W
PCA9955REG_BIT_RAMP_UP_ENABLE = const(7)  # R/W

PCA9955REG_STEP_TIME_GRP0 = const(0x29)  # R/W
# ---------------------------------------------------------------#
#    7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#    -   |CYCTIME|           FACTOR PER STEP                     | (GRP1 - GRP3 repeats)
# ---------------------------------------------------------------#
PCA9955REG_BIT_FACTOR_PER_STEP = const(0)  # R/W
PCA9955REG_BIT_CYCLE_TIME = const(6)  # R/W

PCA9955REG_HOLD_CNTL_GRP0 = const(0x2A)  # R/W
# ---------------------------------------------------------------#
#    7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#  HOLDON|HOLDOFF|      HOLD ON TIME     |     HOLD OFF TIME     | (GRP1 - GRP3 repeats)
# ---------------------------------------------------------------#
PCA9955REG_BIT_HOLD_OFF_TIME = const(0)  # R/W
PCA9955REG_BIT_HOLD_ON_TIME = const(3)  # R/W
PCA9955REG_BIT_HOLD_OFF_ENABLE = const(6)  # R/W
PCA9955REG_BIT_HOLD_ON_ENABLE = const(7)  # R/W

PCA9955REG_IREF_GRP0 = const(0x2B)  # R/W  (GRP1 - GRP3 repeats)
PCA9955REG_GRAD_CNTL = const(0x3E)  # R/W
# ---------------------------------------------------------------#
#    7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#  START3| CONT3 | START2| CONT2 | START1| CONT1 | START0| CONT0 |
# ---------------------------------------------------------------#


# Bit Masks
_1_BIT = const(0b00000001)
_3_BITS = const(0b00000111)
_6_BITS = const(0b00111111)


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
            PCA9955REG_RAMP_RATE_GRP0,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG_BIT_RAMP_UP_ENABLE,
        )

    @ramp_up.setter
    def ramp_up(self, value: bool) -> bool:
        self._device.write_register(
            PCA9955REG_RAMP_RATE_GRP0,
            value,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG_BIT_RAMP_UP_ENABLE,
        )

    @property
    def ramp_down(self) -> bool:
        """Ramp-down enable/disable."""
        return self._device.read_register(
            PCA9955REG_RAMP_RATE_GRP0,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG_BIT_RAMP_DOWN_ENABLE,
        )

    @ramp_down.setter
    def ramp_down(self, value: bool) -> bool:
        self._device.write_register(
            PCA9955REG_RAMP_RATE_GRP0,
            value,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG_BIT_RAMP_DOWN_ENABLE,
        )

    @property
    def ramp_rate(self) -> int:
        """Ramp rate per step 0 - 64."""
        return self._device.read_register(
            PCA9955REG_RAMP_RATE_GRP0,
            offset=self._index,
            mask=_6_BITS,
            bit_offset=PCA9955REG_BIT_RAMP_RATE,
        )

    @ramp_rate.setter
    def ramp_rate(self, value: int) -> int:
        if not 0 <= value <= 64:
            raise ValueError("Value must be between 0 & 64")
        self._device.write_register(
            PCA9955REG_RAMP_RATE_GRP0,
            value,
            offset=self._index,
            mask=_6_BITS,
            bit_offset=PCA9955REG_BIT_RAMP_RATE,
        )

    @property
    def cycle_time(self) -> int:
        """Cycle time - 0 (0.5ms) or 1 (8ms)."""
        return self._device.read_register(
            PCA9955REG_STEP_TIME_GRP0,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG_BIT_CYCLE_TIME,
        )

    @cycle_time.setter
    def cycle_time(self, value: int) -> int:
        if not 0 <= value <= 1:
            raise ValueError("Valid values are 0 (0.5ms) or 1 (8ms)")
        self._device.write_register(
            PCA9955REG_STEP_TIME_GRP0,
            value,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG_BIT_CYCLE_TIME,
        )

    @property
    def factor_per_step(self) -> int:
        """Multiple factor per step 0 - 64."""
        return self._device.read_register(
            PCA9955REG_STEP_TIME_GRP0,
            offset=self._index,
            mask=_6_BITS,
            bit_offset=PCA9955REG_BIT_FACTOR_PER_STEP,
        )

    @factor_per_step.setter
    def factor_per_step(self, value: int) -> int:
        if not 0 <= value <= 64:
            raise ValueError("Value must be between 0 & 64")
        self._device.write_register(
            PCA9955REG_STEP_TIME_GRP0,
            value,
            offset=self._index,
            mask=_6_BITS,
            bit_offset=PCA9955REG_BIT_FACTOR_PER_STEP,
        )

    @property
    def hold_on(self) -> bool:
        """Hold on enable/disable."""
        return self._device.read_register(
            PCA9955REG_HOLD_CNTL_GRP0,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG_BIT_HOLD_ON_ENABLE,
        )

    @hold_on.setter
    def hold_on(self, value: bool) -> bool:
        self._device.write_register(
            PCA9955REG_HOLD_CNTL_GRP0,
            value,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG_BIT_HOLD_ON_ENABLE,
        )

    @property
    def hold_off(self) -> bool:
        """Hold off enable/disable."""
        return self._device.read_register(
            PCA9955REG_HOLD_CNTL_GRP0,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG_BIT_HOLD_OFF_ENABLE,
        )

    @hold_off.setter
    def hold_off(self, value: bool) -> bool:
        self._device.write_register(
            PCA9955REG_HOLD_CNTL_GRP0,
            value,
            offset=self._index,
            mask=_1_BIT,
            bit_offset=PCA9955REG_BIT_HOLD_OFF_ENABLE,
        )

    @property
    def hold_on_time(self) -> int:
        """Hold On time - 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s)
        , 4 (1s), 5 (2s), 6 (4s), 7 (6s)."""
        return self._device.read_register(
            PCA9955REG_HOLD_CNTL_GRP0,
            offset=self._index,
            mask=_3_BITS,
            bit_offset=PCA9955REG_BIT_HOLD_ON_TIME,
        )

    @hold_on_time.setter
    def hold_on_time(self, value: int) -> int:
        if not 0 <= value <= 7:
            raise ValueError(
                "Valid values are 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s)\
                , 4 (1s), 5 (2s), 6 (4s), 7 (6s)"
            )
        self._device.write_register(
            PCA9955REG_HOLD_CNTL_GRP0,
            value,
            offset=self._index,
            mask=_3_BITS,
            bit_offset=PCA9955REG_BIT_HOLD_ON_TIME,
        )

    @property
    def hold_off_time(self) -> int:
        """Hold On time  - 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s)\
        , 4 (1s), 5 (2s), 6 (4s), 7 (6s)."""
        return self._device.read_register(
            PCA9955REG_HOLD_CNTL_GRP0,
            offset=self._index,
            mask=_3_BITS,
            bit_offset=PCA9955REG_BIT_HOLD_OFF_TIME,
        )

    @hold_off_time.setter
    def hold_off_time(self, value: int) -> int:
        if not 0 <= value <= 7:
            raise ValueError(
                "Valid values are 0 (0s), 1 (0.25s), 2 (0.5s), 3 (0.75s)\
                , 4 (1s), 5 (2s), 6 (4s), 7 (6s)"
            )
        self._device.write_register(
            PCA9955REG_HOLD_CNTL_GRP0,
            value,
            offset=self._index,
            mask=_3_BITS,
            bit_offset=PCA9955REG_BIT_HOLD_OFF_TIME,
        )

    @property
    def gain(self) -> int:
        """Group output current gain (0-255)."""
        return self._device.read_register(PCA9955REG_IREF_GRP0, self._index)

    @gain.setter
    def gain(self, value: int) -> int:
        """Set group output current gain to specified value. Should be 0-255."""
        if not 0 <= value <= 255:
            raise ValueError("Valid values are  0-255")
        self._device.write_register(PCA9955REG_IREF_GRP0, value, offset=self._index)

    @property
    def graduation_mode(self) -> bool:
        """Graduation mode. 0 = Single Shot, 1 = Continuous."""
        bit_offset = self._index << 2
        return bool(
            self._device.read_register(PCA9955REG_GRAD_CNTL, mask=_1_BIT, bit_offset=bit_offset)
        )

    @graduation_mode.setter
    def graduation_mode(self, value: bool) -> None:
        """Set graduation mode. 0 = Single Shot, 1 = Continuous."""
        bit_offset = self._index << 2
        self._device.write_register(
            PCA9955REG_GRAD_CNTL, int(value), mask=_1_BIT, bit_offset=bit_offset
        )

    def graduation_start(self) -> None:
        """Start defined Group Graduation running."""
        bit_offset = (self._index << 2) + 1
        self._device.write_register(PCA9955REG_GRAD_CNTL, 0x01, mask=_1_BIT, bit_offset=bit_offset)

    def graduation_stop(self) -> None:
        """Stop Group Graduation."""
        bit_offset = (self._index << 2) + 1
        self._device.write_register(PCA9955REG_GRAD_CNTL, 0x00, mask=_1_BIT, bit_offset=bit_offset)


class Groups:
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
