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

/**@todo add comments here */
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

/** I2C address of the device. */
static uint8_t i2c_address;

/** total coulombCount */
static int64_t coulombCount = 0;

/** total stack voltage */
static uint16_t batVoltage_mV = 0;

/** ADC gain in uV */
static uint16_t adcGain_uV = 0;

/** ADC offset in mV */
static int8_t adcOffset_mV = 0;

static const uint16_t batteryCharge[] =
{
    432, ///<  0.0% charge
    465, ///<  2.5% charge
    500, ///<  5.0% charge
    525, ///<  7.5% charge
    534, ///< 10.0% charge
    536, ///< 12.5% charge
    537, ///< 15.0% charge
    541, ///< 17.5% charge
    546, ///< 20.0% charge
    550, ///< 22.5% charge
    553, ///< 25.0% charge
    555, ///< 27.5% charge
    556, ///< 30.0% charge
    557, ///< 32.5% charge
    558, ///< 35.0% charge
    559, ///< 37.5% charge
    560, ///< 40.0% charge
    561, ///< 42.5% charge
    563, ///< 45.0% charge
    564, ///< 47.5% charge
    566, ///< 50.0% charge
    567, ///< 52.5% charge
    572, ///< 55.0% charge
    576, ///< 57.5% charge
    580, ///< 60.0% charge
    583, ///< 62.5% charge
    586, ///< 65.0% charge
    590, ///< 67.5% charge
    594, ///< 70.0% charge
    597, ///< 72.5% charge
    601, ///< 75.0% charge
    606, ///< 77.5% charge
    610, ///< 80.0% charge
    614, ///< 82.5% charge
    619, ///< 85.0% charge
    624, ///< 87.5% charge
    629, ///< 90.0% charge
    633, ///< 92.5% charge
    639, ///< 95.0% charge
    644, ///< 97.5% charge
    651, ///< 100% charge
};

uint8_t BQ769X0_registerRead(BQ769X0_Register reg);
uint16_t BQ769X0_registerReadWord(BQ769X0_WordRegister reg);
void BQ769X0_registerWrite(BQ769X0_Register reg, uint8_t data);
void BQ769X0_registerWriteWord(BQ769X0_WordRegister reg, uint16_t data);

/** Initialize the BQ769x0 device.
 * @param device Device type that is being used
 */
void BQ769X0_init(BQ769X0_Device device)
{
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

    BQ769X0_wakeup();

    GPIO_INTERRUPT_HIGH_TO_LOW(BQ_FAULT_INT);
    GPIO_INTERRUPT_ENABLE(BQ_FAULT_INT);

    //GPIO_INTERRUPT_HIGH_TO_LOW(BQ_ALERT_INT);
    //GPIO_INTERRUPT_ENABLE(BQ_ALERT_INT);

}

/** Wakeup the BQ769x0 and reinitialize its volatile state.
 */
void BQ769X0_wakeup(void)
{
    GPIO_OUTPUT_SET(THERMISTER_ENABLE);
    BQ769X0_enable();

    BQ769X0_AdcGain1 adcgain1;
    BQ769X0_AdcGain2 adcgain2;

    adcgain1.byte = BQ769X0_registerRead(BQ_ADCGAIN1);
    adcgain2.byte = BQ769X0_registerRead(BQ_ADCGAIN2);

    adcGain_uV = 365 + (adcgain1.adcgain3_4 << 3) + adcgain2.adcgain0_1_2;

    adcOffset_mV = BQ769X0_registerRead(BQ_ADCOFFSET);

    /** @todo not the correct place to do, it should be as soon as we wakeup */
    uint16_t battery_voltage = ADC_convert(ADC_CH_1);
    GPIO_OUTPUT_CLR(THERMISTER_ENABLE);

    uint16_t i;

    for (i = 0; i < sizeof(batteryCharge)/sizeof(batteryCharge[0]); ++i)
    {
        if (battery_voltage <= batteryCharge[i])
        {
            break;
        }
    }

    coulombCount = i * 125;

    BQ769X0_registerWrite(BQ_PROTECT1, BQ769X0_PROTECT1_USER);
    BQ769X0_registerWrite(BQ_PROTECT2, BQ769X0_PROTECT2_USER);
    BQ769X0_registerWrite(BQ_PROTECT3, BQ769X0_PROTECT3_USER);
    BQ769X0_registerWrite(BQ_OV_TRIP, BQ769X0_OV_TRIP_USER);
    BQ769X0_registerWrite(BQ_UV_TRIP, BQ769X0_UV_TRIP_USER);

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

    int32_t voltage = BQ769X0_registerReadWord(BQ_BAT);

    int32_t batVoltage_uV = (4 * voltage * (int32_t)adcGain_uV) +
                            (5 * 1000 * (int32_t)adcOffset_mV);

    batVoltage_mV = (batVoltage_uV + 500) / 1000;

    /* clear any faults */
    BQ769X0_registerWrite(BQ_SYS_STAT, 0xFF);
}

/** Service any pending BQ769x0 tasks.
 */
void BQ769X0_service(void)
{
    BQ769X0_SysStat status;
    status.byte = BQ769X0_registerRead(BQ_SYS_STAT);

    if (status.ccReady)
    {
        BQ769X0_CC cc;

        /* coulomb data ready, update coulomb count */
        cc.word = BQ769X0_registerReadWord(BQ_CC);

        /* Translate to mA, multiply by 1000.
         * Sampled every 250 msec, divide by 4 to normalize to 1 second.
         * Divide by amps/tick to normalize to the sense resistance
         * Nomralize to hours, divide by 3600.
         */
        coulombCount += (cc.cc * 1000) / BQ769X0_CURRENT_SENSE_PER_TICK / 4 / 3600;

        BQ769X0_SysStat clear;
        clear.ccReady = 1;
        BQ769X0_registerWrite(BQ_SYS_STAT, clear.byte);
    }
}

/** Enable and/or wakeup the BQ769x0.
 */
void BQ769X0_enable(void)
{
    GPIO_DIRECTION_OUTPUT(BQ_WAKEUP);
    Timer_spinDelay(15);
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

    BQ769X0_registerWrite(BQ_SYS_CTRL1, sys_ctrl2.byte);
}

/** Stop coulomb counting.
 */
void BQ769X0_coulombCountStop(void)
{
    BQ769X0_SysCtrl2 sys_ctrl2;
    sys_ctrl2.byte = BQ769X0_registerRead(BQ_SYS_CTRL2);

    sys_ctrl2.ccEnable = 0;

    BQ769X0_registerWrite(BQ_SYS_CTRL1, sys_ctrl2.byte);
}

/** Get current coulomb count value.
 * @return current coulomb count value
 */
int64_t BQ769X0_coulombCountGet(void)
{
    return coulombCount;
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

uint16_t BQ769X0_charge_percent(void)
{
    return 0;
}

/** Read a register value on the Bq769x0 device.
 * @param reg register to read
 * @return value read
 */
uint8_t BQ769X0_registerRead(BQ769X0_Register reg)
{
    uint8_t wr_data[1] = {reg};
    uint8_t rd_data[1];

    I2C_sendReceive(BQ769X0_I2C_MODULE, i2c_address, wr_data, 1, rd_data, 1);

    return rd_data[0];
}

/** Read a word (2 byte) register value on the Bq769x0 device.
 * @param reg register to read
 * @return value read
 */
uint16_t BQ769X0_registerReadWord(BQ769X0_WordRegister reg)
{
    uint8_t wr_data[1] = {reg};
    uint8_t rd_data[2];

    I2C_sendReceive(BQ769X0_I2C_MODULE, i2c_address, wr_data, 1, rd_data, 2);

    return ((uint16_t)rd_data[0] << 8) + (uint16_t)rd_data[1];
}

/** Write a register value on the Bq769x0 device.
 * @param reg register to write
 * @param data data to write
 */
void BQ769X0_registerWrite(BQ769X0_Register reg, uint8_t data)
{
    uint8_t wr_data[2] = {reg, data};

    I2C_send(BQ769X0_I2C_MODULE, i2c_address, wr_data, 2);
}

/** Write a word (2 byte) register value on the Bq769x0 device.
 * @param reg register to write
 * @param data data to write
 */
void BQ769X0_registerWriteWord(BQ769X0_WordRegister reg, uint16_t data)
{
    uint8_t wr_data[3] = {reg, data >> 8, data & 0xFF};

    I2C_send(BQ769X0_I2C_MODULE, i2c_address, wr_data, 3);
}
