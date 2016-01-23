/**
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

#endif /* BQ769X0_H_ */
