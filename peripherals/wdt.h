/**
 * @file watchdog.h
 * Management of the watchdog timer.
 *
 * @author Stuart W. Baker
 * @date 4 January 2016
 */

#ifndef INCLUDE_WATCHDOG_H_
#define INCLUDE_WATCHDOG_H_

#include <msp430.h>

/** Initialize Watchdog.
 */
inline void WDT_init(void)
{
    //WDTCTL = WDT_ARST_16;
}

/** reset the watchdog timeout.
 */
inline void WDT_kick(void)
{
    WDT_init();
}

/** Extend the watchdog to the maximum period.
 * @return old watchdog setting for passing to watchdog_restore()
 */
inline uint16_t WDT_extend(void)
{
    uint16_t previous = WDTCTL & 0xFF;
    //WDTCTL = WDT_ARST_1000;
    return previous;
}

/** Restore watchdog to previous period.
 * @param previous pervious period metadata returned by watchdog_extend()
 */
inline void WDT_restore(uint16_t previous)
{
    //WDTCTL = WDTPW & previous;
}

#endif /* INCLUDE_WATCHDOG_H_ */
