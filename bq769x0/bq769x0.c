/**
 * @copyright BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @file bq769x0.c
 * BQ769x0 methods.
 *
 * @author Stuart W. Baker
 * @date 15 April 2015
 */

#include "bq769x0.h"
#include "user.h"
#include "msp430_libs_user.h"
#include "timer.h"
#include "gpio.h"
#include "led.h"

/* These are the various I2C addresses for the various skew of BQ769X0 */
#define BQ769X000_I2C_ADDR 0x08
#define BQ769X001_I2C_ADDR 0x08
#define BQ769X002_I2C_ADDR 0x08
#define BQ769X003_I2C_ADDR 0x08
#define BQ769X006_I2C_ADDR 0x18

/** All the word alligned registers.
 */
typedef enum
{
    BQ_VC1       = 0x0C,
    BQ_VC2       = 0x0E,
    BQ_VC3       = 0x10,
    BQ_VC4       = 0x12,
    BQ_VC5       = 0x14,
    BQ_VC6       = 0x16,
    BQ_VC7       = 0x18,
    BQ_VC8       = 0x1A,
    BQ_VC9       = 0x1C,
    BQ_VC10      = 0x1E,
    BQ_VC11      = 0x20,
    BQ_VC12      = 0x22,
    BQ_VC13      = 0x24,
    BQ_VC14      = 0x26,
    BQ_VC15      = 0x28,
    BQ_BAT       = 0x2A,
    BQ_TS1       = 0x2C,
    BQ_TS2       = 0x2E,
    BQ_TS3       = 0x30,
    BQ_CC        = 0x32,
} BQ769X0_WordRegister;

/** All the byte alligned registers.
 */
typedef enum
{
    BQ_SYS_STAT  = 0x00,
    BQ_CELLBAL1  = 0x01,
    BQ_CELLBAL2  = 0x02,
    BQ_CELLBAL3  = 0x03,
    BQ_SYS_CTRL1 = 0x04,
    BQ_SYS_CTRL2 = 0x05,
    BQ_PROTECT1  = 0x06,
    BQ_PROTECT2  = 0x07,
    BQ_PROTECT3  = 0x08,
    BQ_OV_TRIP   = 0x09,
    BQ_UV_TRIP   = 0x0A,
    BQ_CC_CFG    = 0x0B,
    BQ_VC1_HI    = 0x0C,
    BQ_VC1_LO    = 0x0D,
    BQ_VC2_HI    = 0x0E,
    BQ_VC2_LO    = 0x0F,
    BQ_VC3_HI    = 0x10,
    BQ_VC3_LO    = 0x11,
    BQ_VC4_HI    = 0x12,
    BQ_VC4_LO    = 0x13,
    BQ_VC5_HI    = 0x14,
    BQ_VC5_LO    = 0x15,
    BQ_VC6_HI    = 0x16,
    BQ_VC6_LO    = 0x17,
    BQ_VC7_HI    = 0x18,
    BQ_VC7_LO    = 0x19,
    BQ_VC8_HI    = 0x1A,
    BQ_VC8_LO    = 0x1B,
    BQ_VC9_HI    = 0x1C,
    BQ_VC9_LO    = 0x1D,
    BQ_VC10_HI   = 0x1E,
    BQ_VC10_LO   = 0x1F,
    BQ_VC11_HI   = 0x20,
    BQ_VC11_LO   = 0x21,
    BQ_VC12_HI   = 0x22,
    BQ_VC12_LO   = 0x23,
    BQ_VC13_HI   = 0x24,
    BQ_VC13_LO   = 0x25,
    BQ_VC14_HI   = 0x26,
    BQ_VC14_LO   = 0x27,
    BQ_VC15_HI   = 0x28,
    BQ_VC15_LO   = 0x29,
    BQ_BAT_HI    = 0x2A,
    BQ_BAT_LO    = 0x2B,
    BQ_TS1_HI    = 0x2C,
    BQ_TS1_LO    = 0x2D,
    BQ_TS2_HI    = 0x2E,
    BQ_TS2_LO    = 0x2F,
    BQ_TS3_HI    = 0x30,
    BQ_TS3_LO    = 0x31,
    BQ_CC_HI     = 0x32,
    BQ_CC_LO     = 0x33,
    BQ_ADCGAIN1  = 0x50,
    BQ_ADCOFFSET = 0x51,
    BQ_ADCGAIN2  = 0x59,
} BQ769X0_Register;

/** current system status */
typedef struct
{
    union
    {
        uint8_t byte; ///< full register byte value
        struct
        {
            uint8_t ocd          : 1; ///< over current in discharge fault indicator
            uint8_t scd          : 1; ///< short circuit in discharge fault indicator
            uint8_t ov           : 1; ///< overvoltage fault event indicator
            uint8_t uv           : 1; ///< undervoltage fault event indicator
            uint8_t ovReadAlert  : 1; ///< external override detected
            uint8_t DeviceXReady : 1; ///< internl chip fault detection
            uint8_t reserved     : 1; ///< reserved
            uint8_t ccReady      : 1; ///< fresh coulomb counter reading available
        };
    };
} BQ769X0_SysStat;

/** system control 1 */
typedef struct
{
    union
    {
        uint8_t byte; ///< full register byte value
        struct
        {
            uint8_t shutAB      : 2; ///< shutdown command from host
            uint8_t reserved1   : 1; ///< reserved
            uint8_t tempSel     : 1; ///< temperature source, 0 = internal, 1 = external
            uint8_t adcEnable   : 1; ///< enable temparture and over voltage ADC readings
            uint8_t reserved2   : 2; ///< reserved
            uint8_t loadPresent : 1; ///< 1 = CHG pin detected to exceed VLOAD_DETECT
        };
    };
} BQ769X0_SysCtrl1;

/** system control 2 */
typedef struct
{
    union
    {
        uint8_t byte; ///< full register byte value
        struct
        {
            uint8_t chgOn     : 1; ///< charge on/off signal
            uint8_t dsgOn     : 1; ///< dischage on/off signal
            uint8_t reserved  : 3; ///< reserved
            uint8_t ccOneShot : 1; ///< coulomb counter single reading
            uint8_t ccEnable  : 1; ///< coulomb counter continuous operation
            uint8_t delayDis  : 1; ///< disable delays for faster production testing
        };
    };
} BQ769X0_SysCtrl2;

/** Protect 1 */
typedef struct
{
    union
    {
        uint8_t byte; ///< full register byte value
        struct
        {
            uint8_t scdT     : 3; ///< short circuit in discharge threashold
            uint8_t scdD     : 2; ///< short circuit in discharge delay
            uint8_t reserved : 2; ///< reserved
            uint8_t rsns     : 1; ///< 0 = lower input range, 1 = upper input range
        };
    };
} BQ769X0_Protect1;

/** Protect 2 */
typedef struct
{
    union
    {
        uint8_t byte; ///< full register byte value
        struct
        {
            uint8_t ocdT     : 4; ///< overcurrent in discharge threashold
            uint8_t ocdD     : 3; ///< overcurrent in discharge delay
            uint8_t reserved : 1; ///< reserved
        };
    };
} BQ769X0_Protect2;

/** Protect 3 */
typedef struct
{
    union
    {
        uint8_t byte; ///< full register byte value
        struct
        {
            uint8_t reserved : 4; ///< reserved
            uint8_t ovD      : 2; ///< overvoltage delay
            uint8_t uvD      : 2; ///< undervoltage delay
        };
    };
} BQ769X0_Protect3;

/** CC */
typedef struct
{
    union
    {
        uint16_t word; ///< raw value as read
        int16_t cc; ///< signed data
    };
} BQ769X0_CC;

/** ADC Gain 1 */
typedef struct
{
    union
    {
        uint8_t byte; ///< full register byte value
        struct
        {
            uint8_t reserved1  : 2; ///< reserved
            uint8_t adcgain3_4 : 2; ///< ADC gain bits 3 and 4
            uint8_t reserved2  : 4; ///< undervoltage delay
        };
    };
} BQ769X0_AdcGain1;

/** ADC Gain 2 */
typedef struct
{
    union
    {
        uint8_t byte; ///< full register byte value
        struct
        {
            uint8_t reserved     : 5; ///< reserved
            uint8_t adcgain0_1_2 : 2; ///< ADC gain bits 0, 1, and 2
        };
    };
} BQ769X0_AdcGain2;

/** ADC Offset */
typedef struct
{
    union
    {
        uint8_t byte; ///< full register byte value
        int8_t offset; ///< signed value
    };
} BQ769X0_AdcOffset;

/** total nAH */
/*static*/ int64_t nAH = 0;

/** current in amps averaged over 250 msec */
static int16_t current;

/** ADC gain in uV */
static uint16_t adcGain_uV = 0;

/** I2C address of the device. */
static uint8_t i2c_address;

/** ADC offset in mV */
static int8_t adcOffset_mV = 0;

/** present charging status as determined by the coulomb counter sign */
static bool isCharging = false;

/** we currently have cell ballencing enabled */
static bool isBalancing = false;

/** schedule a rebasing operation */
static bool rebase = false;

/** lockout rebasing operations */
static bool rebaseLockout = false;

void (*sleepResetTimeout)(void);

/** This is essentially the number of points in the batteryCharge table + 1
 */
#define PERCENT_CHARGE_TABLE_FACTOR (BATTERY_MAH_CAPACITY_USER / \
    ((sizeof(BQ769X0_batteryChargeUser) / \
    sizeof(BQ769X0_batteryChargeUser[0])) - 1))

static uint16_t adc_voltage_calibrated(uint16_t counts);
static uint8_t BQ769X0_registerRead(BQ769X0_Register reg);
static uint16_t BQ769X0_registerReadWord(BQ769X0_WordRegister reg);
static void BQ769X0_registerWrite(BQ769X0_Register reg, uint8_t data);
#if 0
static void BQ769X0_registerWriteWord(BQ769X0_WordRegister reg, uint16_t data);
#endif
static void BQ769X0_rebase(bool only_on_full);
static void BQ769X0_rebaseTest(void);
static void BQ769X0_cellBalanceTest(void);
static BQ769X0_CellBalance BQ769X0_cellBalanceIndexTranslate(int index);
static int64_t BQ769X0_getCoulombCount(void);

/** Initialize the BQ769x0 device.
 * @param device Device type that is being used
 */
void BQ769X0_init(BQ769X0_Device device, void (*sleep_reset_timeout)(void))
{
    sleepResetTimeout = sleep_reset_timeout;
    switch (device)
    {
        default:
            break;
        case BQ7692000:
        case BQ7693000:
        case BQ7694000:
            i2c_address = BQ769X000_I2C_ADDR;
            break;
        case BQ7692001:
        case BQ7693001:
        case BQ7694001:
            i2c_address = BQ769X001_I2C_ADDR;
            break;
        case BQ7694002:
        case BQ7692002:
        case BQ7693002:
            i2c_address = BQ769X002_I2C_ADDR;
            break;
        case BQ7692003:
        case BQ7693003:
        case BQ7694003:
            i2c_address = BQ769X003_I2C_ADDR;
            break;
        case BQ7692006:
        case BQ7693006:
        case BQ7694006:
            i2c_address = BQ769X006_I2C_ADDR;
            break;
    }

    /* rebase on the first time startup */
    rebase = true;

    BQ769X0_wakeup();
}

/** Wakeup the BQ769x0 and reinitialize its volatile state.
 */
void BQ769X0_wakeup(void)
{
    gpio_enable_voltage_and_temp();
    BQ769X0_enable();

    BQ769X0_AdcGain1 adc_gain1;
    BQ769X0_AdcGain2 adc_gain2;
    BQ769X0_AdcOffset adc_offset;

    adc_gain1.byte = BQ769X0_registerRead(BQ_ADCGAIN1);
    adc_gain2.byte = BQ769X0_registerRead(BQ_ADCGAIN2);

    adc_offset.byte = BQ769X0_registerRead(BQ_ADCOFFSET);

    adcGain_uV = 365 + (adc_gain1.adcgain3_4 << 3) + adc_gain2.adcgain0_1_2;
    adcOffset_mV = adc_offset.offset;

    /** @todo not the correct place to do, it should be as soon as we wakeup */
    uint16_t battery_voltage = ADC_convert(MSP430_ADC_BATTERY_VOLTAGE_USER);
    gpio_disable_voltage_and_temp();

    /* update volatile protection registers */
    BQ769X0_registerWrite(BQ_PROTECT1, BQ769X0_PROTECT1_USER);
    BQ769X0_registerWrite(BQ_PROTECT2, BQ769X0_PROTECT2_USER);
    BQ769X0_registerWrite(BQ_PROTECT3, BQ769X0_PROTECT3_USER);
    BQ769X0_registerWrite(BQ_OV_TRIP, BQ769X0_OV_TRIP_USER);
    BQ769X0_registerWrite(BQ_UV_TRIP, 0);

    /* The CC_CFG register, per the datasheet, should always be 0x19 */
    BQ769X0_registerWrite(BQ_CC_CFG, 0x19);

    /* - Enable ADC
     * - Temperature reading uses external thermister
     */
    BQ769X0_registerWrite(BQ_SYS_CTRL1, 0x18);

    /** @todo (Stuart Baker) may want to remove continuous coulomb counter */
    /* - Normal delay settings
     * - continuous coulomb counter
     * - discharge signal on
     * - charge signal off
     */
    BQ769X0_registerWrite(BQ_SYS_CTRL2, 0x42);

    /* clear any faults */
    BQ769X0_registerWrite(BQ_SYS_STAT, 0xFF);

    /* check if we need to rebase */
    //BQ769X0_rebaseTest();
}

/** Service any pending BQ769x0 tasks.
 */
void BQ769X0_service(void)
{
    static unsigned int count = 0;
    static bool is_charging_last = false;
    LED_testRedOn();
    BQ769X0_SysStat status;
    status.byte = BQ769X0_registerRead(BQ_SYS_STAT);
    LED_testRedOff();

    if (!status.ccReady)
    {
        /* ~250 msec timer has not expired */
        return;
    }

    BQ769X0_rebaseTest();

    nAH += BQ769X0_getCoulombCount();

    /* make sure we truncate at 100% charged */
    if (nAH > (BATTERY_MAH_CAPACITY_USER * 1000LL * 1000LL))
    {
        nAH = BATTERY_MAH_CAPACITY_USER * 1000LL * 1000LL;
    }
    else if (nAH < 0)
    {
        nAH = 0;
        rebaseLockout = true;
    }

    BQ769X0_SysStat clear;
    clear.ccReady = 1;
    BQ769X0_registerWrite(BQ_SYS_STAT, clear.byte);

    /* we only only test for cell balancing periodically to save processing
     * time.
     *
     * If we are not charging, we will also see if we need to turn of balancing
     * just in case
     */
    if (count++ >= (BQ769X0_CELL_BALANCE_TEST_PERIOD_USER * 4) || !isCharging)
    {
        BQ769X0_cellBalanceTest();
        count = 0;
    }

    /* test if our charging state has changed from charging to not charging */
    if (!isCharging && is_charging_last)
    {
        /* we stopped charging, rebase if the pack is full */
        BQ769X0_rebase(true);
    }
    is_charging_last = isCharging;
}

/** Enable and/or wakeup the BQ769x0.
 */
void BQ769X0_enable(void)
{
    GPIO_DIRECTION_OUTPUT(BQ_WAKEUP);
    Timer_spinDelay(11);
    GPIO_DIRECTION_INPUT(BQ_WAKEUP);
    Timer_spinDelay(2);
}

/** Start coulomb counting.
 */
void BQ769X0_coulombCountStart(void)
{
    BQ769X0_SysCtrl2 sys_ctrl2;
    sys_ctrl2.byte = BQ769X0_registerRead(BQ_SYS_CTRL2);

    sys_ctrl2.ccEnable = 1;

    BQ769X0_registerWrite(BQ_SYS_CTRL2, sys_ctrl2.byte);
}

/** Stop coulomb counting.
 */
void BQ769X0_coulombCountStop(void)
{
    BQ769X0_SysCtrl2 sys_ctrl2;
    sys_ctrl2.byte = BQ769X0_registerRead(BQ_SYS_CTRL2);

    sys_ctrl2.ccEnable = 0;

    BQ769X0_registerWrite(BQ_SYS_CTRL2, sys_ctrl2.byte);
}

/** Turn on/off under voltage tracking.
 * @param enabled true to enable under voltage tracking, else false
 */
void BQ769X0_underVoltageTracking(bool enabled)
{
    static bool enabled_last = false;

    if (enabled && !enabled_last)
    {
        BQ769X0_registerWrite(BQ_UV_TRIP, BQ769X0_UV_TRIP_USER);
    }
    else if (!enabled && enabled_last )
    {
        BQ769X0_registerWrite(BQ_UV_TRIP, 0);
    }
    enabled_last = enabled;
}
/** Enter "SHIP" mode, aka: put the device to sleep.
 */
void BQ769X0_enterShipMode(void)
{
    BQ769X0_SysCtrl1 sys_ctrl1;
    sys_ctrl1.byte = BQ769X0_registerRead(BQ_SYS_CTRL1);

    /* special sequence required to go into shutdown */
    sys_ctrl1.shutAB = 0x0;
    BQ769X0_registerWrite(BQ_SYS_CTRL1, sys_ctrl1.byte);
    sys_ctrl1.shutAB = 0x1;
    BQ769X0_registerWrite(BQ_SYS_CTRL1, sys_ctrl1.byte);
    sys_ctrl1.shutAB = 0x2;
    BQ769X0_registerWrite(BQ_SYS_CTRL1, sys_ctrl1.byte);
}

/** Turn on/off the charge signal.
 * @param active true to turn on, false to turn off
 */
void BQ769X0_chargeSignal(bool active)
{
    BQ769X0_SysCtrl2 sys_ctrl2;
    sys_ctrl2.byte = BQ769X0_registerRead(BQ_SYS_CTRL2);

    sys_ctrl2.chgOn = active ? 1 : 0;

    BQ769X0_registerWrite(BQ_SYS_CTRL2, sys_ctrl2.byte);
}

/** Turn on/off the discharge signal.
 * @param active true to turn on, false to turn off
 */
void BQ769X0_dischargeSignal(bool active)
{
    BQ769X0_SysCtrl2 sys_ctrl2;
    sys_ctrl2.byte = BQ769X0_registerRead(BQ_SYS_CTRL2);

    sys_ctrl2.dsgOn = active ? 1 : 0;

    BQ769X0_registerWrite(BQ_SYS_CTRL2, sys_ctrl2.byte);
}

/** Turn on cell balancing for one particular cell.  Caution, cell balancing
 * should only be used on one cell at a time.
 */
void BQ769X0_cellBalanceOn(BQ769X0_CellBalance cell)
{
    BQ769X0_registerWrite(BQ_CELLBAL1, cell);
}

/** Get the current charge percentage.
 * @return charge percentage (0-100%)
 */
uint8_t BQ769X0_chargePercent(void)
{
    int64_t percent;
    percent = nAH / (BATTERY_MAH_CAPACITY_USER * 1000LL * 1000LL / 100LL);

    if (percent < 0)
    {
        percent = 0;
    }
    else if (percent > 100)
    {
        percent = 100;
    }
    return percent;
}

/** Determine if the battery is charging (positive coulumn count).
 * @return true if charging, else false
 */
bool BQ769X0_isCharging(void)
{
    return isCharging;
}

/** Get the current of the battery averaged over a 250 msec period
 * @return current in amps
 */
int16_t BQ769X0_current(void)
{
    return current;
}

/** Request that the BQ769X0 perform a rebase at the next opportunity.
 */
void BQ769X0_rebaseRequest(void)
{
    rebase = true;
}

/** Convert the 14-bit ADC reading to a calibrated mV reading.
 * @param counts ADC counts
 * @return calibrated ADC value in mV
 */
static uint16_t adc_voltage_calibrated(uint16_t counts)
{
    /* factor in gain */
    int32_t result = (uint32_t)counts * adcGain_uV;

    /* round up and divide by 1000 to get mV */
    result += 500;
    result /= 1000;

    /* factor in offset */
    result += adcOffset_mV;

    return result;
}

/** Get a precision battery voltage measurement and rebase the coulomb counter.
 * @param only_on_full only rebase if pack voltage is above
 */
static void BQ769X0_rebase(bool only_on_full)
{
    uint16_t i;
    uint16_t battery_voltage = BQ769X0_registerReadWord(BQ_BAT);

    if (only_on_full)
    {
        if (battery_voltage < (BQ769X0_MV_CELL_FULL_CHARGE_USER * BQ769X0_CELL_COUNT))
        {
            /* battery not full, don't rebase */
            return;
        }
    }
    for (i = 0; i < (sizeof(BQ769X0_batteryChargeUser)/sizeof(BQ769X0_batteryChargeUser[0]) - 1); ++i)
    {
        if (battery_voltage <= BQ769X0_batteryChargeUser[i])
        {
            break;
        }
    }

    nAH = i * PERCENT_CHARGE_TABLE_FACTOR;
    nAH *= 1000LL * 1000LL;
}

/** Test if a rebase is required, and perform it if so.
 */
static void BQ769X0_rebaseTest(void)
{
    if (rebase)
    {
        /* make sure that we did not lock out our rebasing.  The rebase lockout
         * is cleared when we start charging next.
         */
        if (!rebaseLockout)
        {
            BQ769X0_rebase(false);
        }
        rebase = false;
    }
}

/** Determine if cell balancing is reqiured, and update cell balancing state
 * if necessary.
 */
static void BQ769X0_cellBalanceTest(void)
{
    if (!isCharging)
    {
        if (isBalancing)
        {
            BQ769X0_cellBalanceOn(
                BQ769X0_cellBalanceIndexTranslate(BQ_CELL_NONE_BALANCE));
            isBalancing = false;
        }
        return;
    }

    /* check for cell balancing */
    uint16_t cell_voltage[BQ769X0_CELL_COUNT];
    for (int i = 0, j = 0; i < BQ769X0_CELL_COUNT; ++i, ++j)
    {
#if BQ769X0_CELL_COUNT == 3
        if (i == 2)
        {
            /* skip VC3-VC2 and VC4-VC3 */
            j += 2;
        }
#elif BQ769X0_CELL_COUNT == 4
        if (i == 3)
        {
            /* skip VC4-VC3 */
            ++j;
        }
#elif BQ769X0_CELL_COUNT == 6
        if (i == 2 || i == 5)
        {
            /* skip VC3-VC2 and VC4-VC3 */
            j += 2;
        }
#elif BQ769X0_CELL_COUNT == 7
        if (i == 3)
        {
            /* skip VC3-VC2 and VC4-VC3 */
            ++j;
        }
        if (i == 6)
        {
            /* skip VC3-VC2 and VC4-VC3 */
            j += 2;
        }
#elif BQ769X0_CELL_COUNT == 8
        if (i == 3 || i == 7)
        {
            /* skip VC3-VC2 and VC4-VC3 */
            ++j;
        }
#elif BQ769X0_CELL_COUNT == 9
        if (i == 8)
        {
            /* skip VC3-VC2 and VC4-VC3 */
            ++j;
        }
#endif
        /** @todo add support for up to 15 cells */

        /* read cell voltage */
        int read_index = BQ_VC1 + (j * 2);
        cell_voltage[i] =
            BQ769X0_registerReadWord((BQ769X0_WordRegister)read_index);
    }
    static int old_highest = 0;
    int highest = 0;
    int lowest = 0;

    /* find the highest and lowest cell indexes */
    for (int i = 1; i < BQ769X0_CELL_COUNT; ++i)
    {
        if (cell_voltage[i] > cell_voltage[highest])
        {
            highest = i;
        }
        if (cell_voltage[i] < cell_voltage[lowest])
        {
            lowest = i;
        }
    }

    int balance_index;
    if (cell_voltage[highest] < BQ769X0_MV_CELL_BALANCE_MIN_USER)
    {
        if (!isBalancing)
        {
            /* not above minimum voltage to start ballancing, bail */
            return;
        }

        /* should never get to this point, but just in case, turn
         * off cell ballancing
         */
        balance_index = -1;
    }
    else if ((cell_voltage[highest] - cell_voltage[lowest]) <=
        BQ769X0_MV_CELL_BALANCE_STOP_USER && isBalancing)
    {
        /* we are balancing and we have crossed our stop balancing
         * threashold
         */
        balance_index = -1;
    }
    else if ((isBalancing && highest != old_highest) ||
             ((cell_voltage[highest] - cell_voltage[lowest]) >=
              BQ769X0_MV_CELL_BALANCE_START_USER && !isBalancing))
    {
        /* we are balancing, but have a new highest voltage cell
         *
         * ...or...
         *
         * we are not balancing and we have crossed our start balancing
         * threashold
         */
        balance_index = old_highest = highest;
    }
    else
    {
        return;
    }

    /* update balancing state */
    BQ769X0_cellBalanceOn(BQ769X0_cellBalanceIndexTranslate(balance_index));
    isBalancing = balance_index == BQ_CELL_NONE_BALANCE ? false : true;
}

/** Get the last coulomb count.
 * @return coulomb count change in nAH
 */
static int64_t BQ769X0_getCoulombCount(void)
{
    BQ769X0_CC cc;

    /* coulomb data ready, update coulomb count */
    cc.word = BQ769X0_registerReadWord(BQ_CC);

    /* filter out some noise */
    if (cc.cc > 4 || cc.cc < -25)
    {
        sleepResetTimeout();

        /* setup or charging status flag */
        if (cc.cc > 0)
        {
            isCharging = true;
            rebaseLockout = false;
        }
        else
        {
            //LED_testRedOn();
            isCharging = false;
        }

        current = cc.cc / BQ769X0_CC_PER_AMP_USER;

        /* Translate to nA, multiply by 1000 * 1000 * 1000.
         * Sampled every 250 msec, divide by 4 to normalize to 1 second.
         * Divide by amps/tick to normalize to the sense resistance
         * Nomralize to hours, divide by 3600.
         */
        return (cc.cc * 1000LL * 1000LL * 1000LL) /
               BQ769X0_CC_PER_AMP_USER / 4 / 3600;
    }

    //LED_testRedOff();
    isCharging = false;
    return 0;
}

/** Translate from the logical configuration index (3 - 15 cells) to the index
 * index and ultimate enumaration used by @ref BQ769X0_CellBalance.
 * @param index logical configuration index
 * @return @ref BQ769X0_CellBalance enumeration coresponding to index
 */
static BQ769X0_CellBalance BQ769X0_cellBalanceIndexTranslate(int index)
{
    int actual_index = index;
#if BQ769X0_CELL_COUNT == 3
                if (index == 2)
                {
                    /* skip VC3-VC2 and VC4-VC3 */
                    actual_index += 2;
                }
#elif BQ769X0_CELL_COUNT == 4
                if (index == 3)
                {
                    /* skip VC4-VC3 */
                    ++actual_index;
                }
#elif BQ769X0_CELL_COUNT == 6
                if (index == 2 || index == 5)
                {
                    /* skip VC3-VC2 and VC4-VC3 */
                    actual_index += 2;
                }
#elif BQ769X0_CELL_COUNT == 7
                if (index == 3)
                {
                    /* skip VC3-VC2 and VC4-VC3 */
                    ++actual_index;
                }
                if (index == 6)
                {
                    /* skip VC3-VC2 and VC4-VC3 */
                    actual_index += 2;
                }
#elif BQ769X0_CELL_COUNT == 8
                if (index == 3 || index == 7)
                {
                    /* skip VC3-VC2 and VC4-VC3 */
                    ++actual_index;
                }
#elif BQ769X0_CELL_COUNT == 9
                if (index == 8)
                {
                    /* skip VC3-VC2 and VC4-VC3 */
                    ++actual_index;
                }
#endif
    switch (actual_index)
    {
        default:
            return BQ_CELL_NONE_BALANCE;
        case 0:
            return BQ_CELL_1_BALANCE;
        case 1:
            return BQ_CELL_2_BALANCE;
        case 2:
            return BQ_CELL_3_BALANCE;
        case 3:
            return BQ_CELL_4_BALANCE;
        case 4:
            return BQ_CELL_5_BALANCE;
    }
}

/** Read a register value on the Bq769x0 device.
 * @param reg register to read
 * @return value read
 */
static uint8_t BQ769X0_registerRead(BQ769X0_Register reg)
{
    uint8_t wr_data[1] = {reg};
    uint8_t rd_data[1];
    //LED_testRedOn();
    I2C_sendReceive(BQ769X0_I2C_MODULE, i2c_address, wr_data, 1, rd_data, 1);
    //LED_testRedOff();
    return rd_data[0];
}

/** Read a word (2 byte) register value on the Bq769x0 device.
 * @param reg register to read
 * @return value read
 */
static uint16_t BQ769X0_registerReadWord(BQ769X0_WordRegister reg)
{
    uint8_t wr_data[1] = {reg};
    uint8_t rd_data[2];
    //LED_testRedOn();
    I2C_sendReceive(BQ769X0_I2C_MODULE, i2c_address, wr_data, 1, rd_data, 2);
    //LED_testRedOff();
    return ((uint16_t)rd_data[0] << 8) + (uint16_t)rd_data[1];
}

/** Write a register value on the Bq769x0 device.
 * @param reg register to write
 * @param data data to write
 */
static void BQ769X0_registerWrite(BQ769X0_Register reg, uint8_t data)
{
    uint8_t wr_data[2] = {reg, data};
    //LED_testRedOn();
    I2C_send(BQ769X0_I2C_MODULE, i2c_address, wr_data, 2);
    //LED_testRedOff();
}

#if 0
/** Write a word (2 byte) register value on the Bq769x0 device.
 * @param reg register to write
 * @param data data to write
 */
static void BQ769X0_registerWriteWord(BQ769X0_WordRegister reg, uint16_t data)
{
    uint8_t wr_data[3] = {reg, data >> 8, data & 0xFF};

    I2C_send(BQ769X0_I2C_MODULE, i2c_address, wr_data, 3);
}
#endif
