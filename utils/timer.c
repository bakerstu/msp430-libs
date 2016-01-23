/**
 * @file timer.h Implements multiple timers using a common hardware timer.
 *
 * @author Stuart W. Baker
 * @date 20 February 2015
 */

#include "timer.h"

#include <msp430.h>
#include <stdlib.h>

#include "watchdog.h"

/** Timer A registers */
typedef struct
{
    volatile uint16_t TAxCTL; /**< Timer A control */
    volatile uint16_t TAxCCTL0; /**< Timer A capture/compare control 0 */
    volatile uint16_t TAxCCTL1; /**< Timer A capture/compare control 1 */
    volatile uint16_t TAxCCTL2; /**< Timer A capture/compare control 2 */
    volatile uint16_t pad[4]; /**< unused padding */
    volatile uint16_t TAxR; /**< Timer A counter */
    volatile uint16_t TAxCCR0; /**< Timer A capture/comapre 0 */
    volatile uint16_t TAxCCR1; /**< Timer A capture/comapre 1 */
    volatile uint16_t TAxCCR2; /**< Timer A capture/comapre 2 */
} TIMER_A_Registers;

/** Timer A instance registers for TIMER instantiation */
static volatile TIMER_A_Registers *registers = NULL;

/** current time */
static uint64_t now = 0;

/** timer list */
Timer *list = NULL;

/** Get the current time in msec since power on.
 * @return current time in msec
 */
uint64_t TIMER_gettime(void)
{
    return now;
}

/** Delay in a spin loop.
 * @param period timer timeout period in msec;
 */
void TIMER_spinDelay(uint32_t period)
{
    uint64_t timeout = now + period;

    while (now < timeout)
    {
        TIMER_service();
    }
}


/** Insert timer into the timer list.
 * @param timer reference to the timer that will be inserted.
 */
static void TIMER_insert(Timer *timer)
{
    Timer *tp = list;
    Timer *last = NULL;

    if (timer->when == 0)
    {
        /* timer not active */
        return;
    }

    while (tp)
    {
        if (timer->when <= tp->when)
        {
            break;
        }
        last = tp;
        tp = tp->next;
    }
    if (last)
    {
        timer->next = last->next;
        last->next = timer;
    }
    else
    {
        timer->next = list;
        list = timer;
    }
}

/** Remove the timer from the list.
 * @param timer reference to the timer that will be removed.
 */
static void TIMER_remove(Timer *timer)
{
    Timer *tp = list;
    Timer *last = NULL;
    if (timer->when == 0)
    {
        /* timer not active */
        return;
    }

    while (tp)
    {
        if (tp == timer)
        {
            break;
        }
        last = tp;
        tp = tp->next;
    }
    if (tp)
    {
        if (last)
        {
            last->next = tp->next;
        }
        else
        {
            list = tp->next;
        }
    }
    timer->when = 0;
}

/** Restart the timer based on a previous period.
 * @param timer reference to the timer that will be restarted.
 */
void TIMER_restart(Timer *timer)
{
    TIMER_remove(timer);

    timer->when = now + timer->period;

    TIMER_insert(timer);
}

/** Stop the timer it it is running.
 * @param timer reference to the timer that will be stopped.
 */
void TIMER_stop(Timer *timer)
{
    TIMER_remove(timer);
}

/** Get the current time remaining before timer expires.
 * @param timer reference to the timer that will be queried.
 * @return time remaining in msec.
 */
uint32_t TIMER_remaining(Timer *timer)
{
    if (timer->when == 0)
    {
        return 0xFFFFFFFF;
    }
    if (now >= timer->when)
    {
        return 0;
    }
    else
    {
        return timer->when - now;
    }
}

/** Service any expired timers.
 */
void TIMER_service(void)
{
    Timer *current = list;

    while (current)
    {
        WDT_kick();

        Timer *timer = current;
        current = current->next;

        if (timer->when <= now)
        {
            /* remove the timer from the active list */
            uint64_t when = timer->when;
            TIMER_remove(timer);

            /* service timer callback */
            uint32_t result = timer->callback(timer->priv);
            switch(result)
            {
                case TIMER_NONE:
                    /* do not restart the timer */
                    break;
                default:
                    /* restart the timer with a new period */
                    timer->period = result;
                    /* fall through */
                case TIMER_RESTART:
                    /* restart the timer */
                    timer->when = when + timer->period;
                    TIMER_insert(timer);
                    break;
            }
            WDT_kick();
            /* restart loop from the beginning in case list has changed */
            current = list;
        }
        else
        {
            /* no more expired timers */
            break;
        }
    }
}

/*
 * TIMER_init
 */
void TIMER_init(uint16_t instance, TIMER_Config *config)
{
    if (registers != NULL)
    {
        /* already initialized */
        return;
    }

    registers = (volatile TIMER_A_Registers*)instance;

    /* stop and clear timer */
    registers->TAxCTL = MC_0 + TACLR;

    /* initialize current time to zero */
    now = 0;

    int divide;
    switch (config->clockSourceDivider)
    {
        default:
        case TIMER_CLOCKSOURCE_DIVIDER_1:
            divide = 1;
            break;
        case TIMER_CLOCKSOURCE_DIVIDER_2:
            divide = 2;
            break;
        case TIMER_CLOCKSOURCE_DIVIDER_4:
            divide = 4;
            break;
        case TIMER_CLOCKSOURCE_DIVIDER_8:
            divide = 8;
            break;
    }
    registers->TAxCCR0 = ((config->timerClk / divide) / config->timerRate) - 1;
    /* turn on timer and enable interrupts */
    registers->TAxCTL = config->clockSource + config->clockSourceDivider +
                        MC_1 + TACLR + TAIE;
}

/** Timer A0 interrupt service routine.
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer_A (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{
    /* clear flag */
    registers->TAxCTL &= ~TAIFG;

    /* increment time */
    ++now;
}
