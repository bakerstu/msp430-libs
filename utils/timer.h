/**
 * @file timer.h Implements multiple timers using a common hardware timer.
 *
 * @author Stuart W. Baker
 * @date 20 February 2015
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>
#include <stdbool.h>

/** Base address of TIMER module Registers */
#define TIMER_0_MODULE ((uint16_t)&TA0CTL)

/** Base address of TIMER module Registers */
#define TIMER_1_MODULE ((uint16_t)&TA1CTL)

/** Divide source clock by 1 */
#define TIMER_CLOCKSOURCE_DIVIDER_1 (ID_0)

/** Divide source clock by 2 */
#define TIMER_CLOCKSOURCE_DIVIDER_2 (ID_1)

/** Divide source clock by 4 */
#define TIMER_CLOCKSOURCE_DIVIDER_4 (ID_2)

/** Divide source clock by 8 */
#define TIMER_CLOCKSOURCE_DIVIDER_8 (ID_3)

/** Clock source TACLK */
#define TIMER_A_CLOCK_SOURCE_TACLK (TASSEL_0)

/** Clock source ACLK */
#define TIMER_A_CLOCK_SOURCE_ACLK  (TASSEL_1)

/** Clock source SMCLK */
#define TIMER_A_CLOCK_SOURCE_SMCLK (TASSEL_2)

/** Clock source INCLK */
#define TIMER_A_CLOCK_SOURCE_INCLK (TASSEL_3)

/** Configuration parameters for the Timer.
 */
typedef struct
{
    /** Clock source.  Valid values are
     * - \b TIMER_A_CLOCK_SOURCE_TACLK
     * - \b TIMER_A_CLOCK_SOURCE_ACLK
     * - \b TIMER_A_CLOCK_SOURCE_SMCLK
     * - \b TIMER_A_CLOCK_SOURCE_INCLK
     */
    uint16_t clockSource;
    /** Divide integer for the timer.  Valid values are
     * - \b TIMER_CLOCKSOURCE_DIVIDER_1
     * - \b TIMER_CLOCKSOURCE_DIVIDER_2
     * - \b TIMER_CLOCKSOURCE_DIVIDER_4
     * - \b TIMER_CLOCKSOURCE_DIVIDER_8
     */
    uint16_t clockSourceDivider;
    /** Rate in Hz of the clock source specified in selectClockSource. */
    uint32_t timerClk;
    /** timer tick rate in Hz, typcially 1000. */
    uint32_t timerRate;
} TIMER_Config;

/** Data structure for Timer metadata.
 * This structure should not be accessed directly by an application.  It
 * is considered private.
 */
typedef struct Timer
{
    uint64_t when; /**< when the timer will expire */
    uint32_t period; /**< timeout period in msec */
    void *priv; /**< private data context passed to callback */
    uint32_t (*callback)(void *); /**< timer callback method */
    struct Timer *next; /**< next timer to expire in timer list */
} Timer;

/** Do not restart the timer */
#define TIMER_NONE    0

/** Restart the timer */
#define TIMER_RESTART 0xFFFFFFFF

/** Initialize TIMER.
 *
 * @param instance Instance the instance of the TIMER module.
 *                       Valid values include:
 *                        - \b TIMER_0_MODULE
 *                        - \b TIMER_1_MODULE
 * @param config Confugration structure for the TIMER.
 */
void TIMER_init(uint16_t instance, TIMER_Config *config);

/** Create and initialize the timer.
 * @param timer reference to the timer that will be initialized.
 * @param callback timer user application callback
 * @param priv private data for callback
 */
inline void TIMER_create(Timer *timer, uint32_t (*callback)(void *), void *priv)
{
    timer->when = 0;
    timer->priv = priv;
    timer->callback = callback;
}

/** Restart the timer based on a previous period.
 * @param timer reference to the timer that will be restarted.
 */
void TIMER_restart(Timer *timer);

/** Test to see if the timer is active.
 * @param timer reference to the timer that will be queried.
 * @return true of timer active, else false
 */
inline bool TIMER_active(Timer *timer)
{
    return timer->when;
}

/** Get the current timer period in msec.
 * @param timer reference to the timer that will be queried.
 * @return current timer period
 */
inline uint32_t TIMER_period(Timer *timer)
{
    return timer->period;
}

/** Get the current time remaining before timer expires.
 * @param timer reference to the timer that will be queried.
 * @return time remaining in msec.
 */
uint32_t TIMER_remaining(Timer *timer);

/** Start the timer.
 * @param timer reference to the timer that will be started.
 * @param period timer timeout period in msec;
 */
inline void TIMER_start(Timer *timer, uint32_t period)
{
    timer->period = period;
    TIMER_restart(timer);
}

/** Stop the timer it it is running.
 * @param timer reference to the timer that will be stopped.
 */
void TIMER_stop(Timer *timer);

/** Service any expired timers.
 */
void TIMER_service(void);

/** Delay in a spin loop.
 * @param period timer timeout period in msec;
 */
void TIMER_spinDelay(uint32_t period);

/** Get the current time in msec since power on.
 * @return current time in msec
 */
uint64_t TIMER_gettime(void);

#endif /* TIMER_H_ */
