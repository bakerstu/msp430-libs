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
 * @file bq769x0.h
 * BQ769x0 methods.  The reference voltage is 6.25V and at 14 bit, it is 382
 * microvolts/count.
 *
 * @author Stuart W. Baker
 * @date 15 April 2015
 */

#ifndef BQ769X0_H_
#define BQ769X0_H_

#include <stdint.h>
#include <stdbool.h>

#include "uscib_i2c.h"

/** Period between each coulomb count read in msec. */
#define BQ769X0_COULOMB_COUNT_PERIOD 250

/** All the different device types covered by this driver.
 */
typedef enum
{
    BQ7692000 = 0,
    BQ7692001,
    BQ7692002,
    BQ7692003,
    BQ7692006,
    BQ7693000,
    BQ7693001,
    BQ7693002,
    BQ7693003,
    BQ7693006,
    BQ7694000,
    BQ7694001,
    BQ7694002,
    BQ7694003,
    BQ7694006,
} BQ769X0_Device;

/** Valid cell balance values.
 */
typedef enum
{
    BQ_CELL_1_BALANCE = 0x01, ///< cell 1 balance
    BQ_CELL_2_BALANCE = 0x02, ///< cell 2 balance
    BQ_CELL_3_BALANCE = 0x04, ///< cell 3 balance
    BQ_CELL_4_BALANCE = 0x08, ///< cell 4 balance
    BQ_CELL_5_BALANCE = 0x10, ///< cell 5 balance
} BQ769X0_CellBalance;

/** Initialize the BQ769x0 device.
 * @param device Device type that is being used
 */
void BQ769X0_init(BQ769X0_Device device);

/** Wakeup the BQ769x0 and reinitialize its volatile state.
 */
void BQ769X0_wakeup(void);

/** Enable and/or wakeup the BQ769x0.
 */
void BQ769X0_enable(void);

/** Service any pending BQ769x0 tasks.
 */
void BQ769X0_service(void);

/** Start coulomb counting.
 */
void BQ769X0_coulombCountStart(void);

/** Stop coulomb counting.
 */
void BQ769X0_coulombCountStop(void);

/** Get current coulomb count value.
 * @return current coulomb count value
 */
int64_t BQ769X0_coulombCountGet(void);

/** Enter "SHIP" mode, aka: put the device to sleep.
 */
void BQ769X0_enterShipMode(void);

/** Turn on/off the charge signal.
 * @param active true to turn on, false to turn off
 */
void BQ769X0_chargeSignal(bool active);

/** Turn on/off the discharge signal.
 * @param active true to turn on, false to turn off
 */
void BQ769X0_dischargeSignal(bool active);

/** Turn on cell balancing for one particular cell.  Caution, cell balancing
 * should only be used on one cell at a time.
 */
void BQ769X0_cellBalanceOn(BQ769X0_CellBalance cell);

/** Get the current charge percentage.
 * @return charge percentage (0-100%)
 */
uint8_t BQ769X0_charge_percent(void);

/** Determine if the battery is charging (positive coulumn count).
 * @return true if charging, else false
 */
bool BQ769X0_is_charging(void);

#endif /* BQ769X0_H_ */
