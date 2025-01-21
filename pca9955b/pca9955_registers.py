# SPDX-FileCopyrightText: Copyright (c) 2024 Noel Anderson
#
# SPDX-License-Identifier: Unlicense
"""
`pca9955b_registers
================================================================================

CircuitPython helper library for the NXP 16-Channel I�C-Bus Constant-Current LED Driver


* Author(s): Noel Anderson

This module defines the register map and bit positions for the PCA9955B LED driver.

Registers:
    MODE1 (0x00): Mode register 1 (R/W)
        - BIT_ALLCALL (0): All Call address enable (R)
        - BIT_SUB3 (1): Subaddress 3 enable (R/W)
        - BIT_SUB2 (2): Subaddress 2 enable (R/W)
        - BIT_SUB1 (3): Subaddress 1 enable (R/W)
        - BIT_SLEEP (4): Sleep mode (R/W)
        - BIT_AI0 (5): Auto-Increment bit 0 (R/W)
        - BIT_AI1 (6): Auto-Increment bit 1 (R/W)
        - BIT_AIF (7): Auto-Increment flag (R/W)

    MODE2 (0x01): Mode register 2 (R/W)
        - BIT_EXP_EN (2): Expansion enable (R/W)
        - BIT_OCH (3): Output change (R/W)
        - BIT_CLRERR (4): Clear error (W)
        - BIT_DMBLNK (5): Dimming blink (R/W)
        - BIT_ERROR (6): Error flag (R)
        - BIT_OVERTEMP (7): Over-temperature flag (R)

    LEDOUT0 (0x02): LED output state register 0 (R/W)
        - BIT_LED0 (0): LED0 state (R/W)
        - BIT_LED1 (2): LED1 state (R/W)
        - BIT_LED2 (4): LED2 state (R/W)
        - BIT_LED3 (6): LED3 state (R/W)

    GRPPWM (0x06): Group duty cycle control (R/W)
    GRPFREQ (0x07): Group frequency control (R/W)

    PWM0 (0x08): PWM control register 0 (R/W)
    IREF0 (0x18): Current reference control register 0 (R/W)

    RAMP_RATE_GRP0 (0x28): Ramp rate control register for group 0 (R/W)
        - BIT_RAMP_RATE (0): Ramp rate (R/W)
        - BIT_RAMP_DOWN_ENABLE (6): Ramp down enable (R/W)
        - BIT_RAMP_UP_ENABLE (7): Ramp up enable (R/W)

    STEP_TIME_GRP0 (0x29): Step time control register for group 0 (R/W)
        - BIT_FACTOR_PER_STEP (0): Factor per step (R/W)
        - BIT_CYCLE_TIME (6): Cycle time (R/W)

    HOLD_CNTL_GRP0 (0x2A): Hold control register for group 0 (R/W)
        - BIT_HOLD_OFF_TIME (0): Hold off time (R/W)
        - BIT_HOLD_ON_TIME (3): Hold on time (R/W)
        - BIT_HOLD_OFF_ENABLE (6): Hold off enable (R/W)
        - BIT_HOLD_ON_ENABLE (7): Hold on enable (R/W)

    IREF_GRP0 (0x2B): Current reference control register for group 0 (R/W)

    GRAD_MODE_SEL0 (0x38): Gradient mode selection register 0 (R/W)
    GRAD_MODE_SEL1 (0x39): Gradient mode selection register 1 (R/W)

    GRAD_GRP_SEL0 (0x3A): Gradient group selection register 0 (R/W)
    GRAD_CNTL (0x3E): Gradient control register (R/W)
        - BIT_CONTINUOUS_0 (0): Continuous mode 0 (R/W)
        - BIT_START_0 (1): Start mode 0 (R/W)
        - BIT_CONTINUOUS_1 (2): Continuous mode 1 (R/W)
        - BIT_START_1 (3): Start mode 1 (R/W)
        - BIT_CONTINUOUS_2 (4): Continuous mode 2 (R/W)
        - BIT_START_2 (5): Start mode 2 (R/W)
        - BIT_CONTINUOUS_3 (6): Continuous mode 3 (R/W)
        - BIT_START_3 (7): Start mode 3 (R/W)

    OFFSET (0x3F): Offset register (R/W)
        - BIT_OUTPUT_DELAY (0): Output delay (R/W)

    SUBADR1 (0x40): Subaddress 1 (R/W)
    SUBADR2 (0x41): Subaddress 2 (R/W)
    SUBADR3 (0x42): Subaddress 3 (R/W)
    ALLCALLADR (0x43): All call address (R/W)
        - BIT_SUBADR (1): Subaddress bit (R/W)

    PWMALL (0x44): PWM control for all LEDs (R/W)
    IREFALL (0x45): Current reference control for all LEDs (R/W)

    EFLAG0 (0x46): Error flag register 0 (R)
"""

from micropython import const

# Register map & bit positions for PCA9955B

MODE1 = const(0x00)  # R/W
# ---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#  AIF  |  AI1  |  AI0  | SLEEP |  SUB1 |  SUB2 |  SUB3 |ALLCALL|
# ---------------------------------------------------------------#
BIT_ALLCALL = const(0)  # R
BIT_SUB3 = const(1)  # R/W
BIT_SUB2 = const(2)  # R/W
BIT_SUB1 = const(3)  # R/W
BIT_SLEEP = const(4)  # R/W
BIT_AI0 = const(5)  # R/W
BIT_AI1 = const(6)  # R/W
BIT_AIF = const(7)  # R/W

MODE2 = const(0x01)  # R/W
# ---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
# OVERTP| ERROR | DMBLNK| CLRERR|  OCH  |EXP_EN |   -   |   -   |
# ---------------------------------------------------------------#
BIT_EXP_EN = const(2)  # R/W
BIT_OCH = const(3)  # R/W
BIT_CLRERR = const(4)  # W
BIT_DMBLNK = const(5)  # R/W
BIT_ERROR = const(6)  # R
BIT_OVERTEMP = const(7)  # R

LEDOUT0 = const(0x02)  # R/W
# LEDOUT1 - LEDOUT3 repeats
# ---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#      LED3     |      LED2     |      LED1     |      LEDO     | LEDOUT0
# -------+-------+-------+-------+-------+-------+-------+-------|
#      LED7     |      LED6     |      LED5     |      LED4     | LEDOUT1
# -------+-------+-------+-------+-------+-------+-------+-------|
#      LED11    |      LED10    |      LED9     |      LED8     | LEDOUT2
# -------+-------+-------+-------+-------+-------+-------+-------|
#      LED15    |      LED14    |      LED13    |      LED12    | LEDOUT3
# ---------------------------------------------------------------#
BIT_LED0 = BIT_LED4 = BIT_LED8 = BIT_LED12 = const(0)  # R/W
BIT_LED1 = BIT_LED5 = BIT_LED9 = BIT_LED13 = const(2)  # R/W
BIT_LED2 = BIT_LED6 = BIT_LED10 = BIT_LED14 = const(4)  # R/W
BIT_LED3 = BIT_LED7 = BIT_LED11 = BIT_LED15 = const(6)  # R/W

GRPPWM = const(0x06)  # R/W
GRPFREQ = const(0x07)  # R/W

PWM0 = const(0x08)  # R/W
# PWM1 - PWM15 repeats

IREF0 = const(0x18)  # R/W
# IREF1 - IREF15 repeats

RAMP_RATE_GRP0 = const(0x28)  # R/W
# GRP1 - GRP3 repeats
# ---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
# RAMPUP| RAMPDW|              RAMP RATE                        |
# ---------------------------------------------------------------#
BIT_RAMP_RATE = const(0)  # R/W
BIT_RAMP_DOWN_ENABLE = const(6)  # R/W
BIT_RAMP_UP_ENABLE = const(7)  # R/W

STEP_TIME_GRP0 = const(0x29)  # R/W
# GRP1 - GRP3 repeats
# ---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#   -   |CYCTIME|           FACTOR PER STEP                     |
# ---------------------------------------------------------------#
BIT_FACTOR_PER_STEP = const(0)  # R/W
BIT_CYCLE_TIME = const(6)  # R/W

HOLD_CNTL_GRP0 = const(0x2A)  # R/W
# GRP1 - GRP3 repeats
# ---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
# HOLDON|HOLDOFF|      HOLD ON TIME     |     HOLD OFF TIME     |
# ---------------------------------------------------------------#
BIT_HOLD_OFF_TIME = const(0)  # R/W
BIT_HOLD_ON_TIME = const(3)  # R/W
BIT_HOLD_OFF_ENABLE = const(6)  # R/W
BIT_HOLD_ON_ENABLE = const(7)  # R/W

IREF_GRP0 = const(0x2B)  # R/W
# GRP1 - GRP3 repeats

GRAD_MODE_SEL0 = const(0x38)  # R/W
GRAD_MODE_SEL1 = const(0x39)  # R/W

GRAD_GRP_SEL0 = const(0x3A)  # R/W
# GRP1 - GRP3 repeats
# ---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#      LED3     |      LED2     |      LED1     |      LEDO     | GRAD_GRP_SEL0
# -------+-------+-------+-------+-------+-------+-------+-------|
#      LED7     |      LED6     |      LED5     |      LED4     | GRAD_GRP_SEL1
# -------+-------+-------+-------+-------+-------+-------+-------|
#      LED11    |      LED10    |      LED9     |      LED8     | GRAD_GRP_SEL2
# -------+-------+-------+-------+-------+-------+-------+-------|
#      LED15    |      LED14    |      LED13    |      LED12    | GRAD_GRP_SEL3
# ---------------------------------------------------------------#

GRAD_CNTL = const(0x3E)  # R/W
# ---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
# START3| CONT3 | START2| CONT2 | START1| CONT1 | START0| CONT0 |
# ---------------------------------------------------------------#
BIT_CONTINUOUS_0 = const(0)  # R/W
BIT_START_0 = const(1)  # R/W
BIT_CONTINUOUS_1 = const(2)  # R/W
BIT_START_1 = const(3)  # R/W
BIT_CONTINUOUS_2 = const(4)  # R/W
BIT_START_2 = const(5)  # R/W
BIT_CONTINUOUS_3 = const(6)  # R/W
BIT_START_3 = const(7)  # R/W

OFFSET = const(0x3F)  # R/W
# ---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#               -               |        OUTPUT_DELAY           |
# ---------------------------------------------------------------#
BIT_OUTPUT_DELAY = const(0)  # R/W

SUBADR1 = const(0x40)  # R/W
SUBADR2 = const(0x41)  # R/W
SUBADR3 = const(0x42)  # R/W
ALLCALLADR = const(0x43)  # R/W
# ---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#                  I2C_BUS_SUBADDRESS                   |   -   |
# ---------------------------------------------------------------#
BIT_SUBADR = const(1)  # R/W

PWMALL = const(0x44)  # R/W
IREFALL = const(0x45)  # R/W

EFLAG0 = const(0x46)  # R
# EFLAG1 - EFLAG3 repeats
# ---------------------------------------------------------------#
#   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
# -------+-------+-------+-------+-------+-------+-------+-------|
#      LED3     |      LED2     |      LED1     |      LEDO     | EFLAG0
# -------+-------+-------+-------+-------+-------+-------+-------|
#      LED7     |      LED6     |      LED5     |      LED4     | EFLAG1
# -------+-------+-------+-------+-------+-------+-------+-------|
#      LED11    |      LED10    |      LED9     |      LED8     | EFLAG2
# -------+-------+-------+-------+-------+-------+-------+-------|
#      LED15    |      LED14    |      LED13    |      LED12    | EFLAG3
# ---------------------------------------------------------------#
