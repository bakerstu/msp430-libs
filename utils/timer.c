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
 * @file timer.h Implements multiple timers using a common hardware timer.
 *
 * @author Stuart W. Baker
 * @date 20 February 2015
 */

#include "timer.h"

#include <msp430.h>
#include <stdlib.h>

#include "wdt.h"

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
} Timer_A_Registers;

/** Timer A instance registers for TIMER instantiation */
static volatile Timer_A_Registers *registers = NULL;

/** current time */
static uint64_t now = 0;

/** timer list */
Timer *list = NULL;

/** Get the current time in msec since power on.
 * @return current time in msec
 */
uint64_t Timer_gettime(void)
{
    return now;
}

/** Delay in a spin loop.
 * @param period timer timeout period in msec;
 */
void Timer_spinDelay(uint32_t period)
{
    uint64_t timeout = now + period;

    while (now < timeout)
    {
        Timer_service();
    }
}


/** Insert timer into the timer list.
 * @param timer reference to the timer that will be inserted.
 */
static void Timer_insert(Timer *timer)
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
static void Timer_remove(Timer *timer)
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
void Timer_restart(Timer *timer)
{
    Timer_remove(timer);

    timer->when = now + timer->period;

    Timer_insert(timer);
}

/** Stop the timer it it is running.
 * @param timer reference to the timer that will be stopped.
 */
void Timer_stop(Timer *timer)
{
    Timer_remove(timer);
}

/** Get the current time remaining before timer expires.
 * @param timer reference to the timer that will be queried.
 * @return time remaining in msec.
 */
uint32_t Timer_remaining(Timer *timer)
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
void Timer_service(void)
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
            Timer_remove(timer);

            /* service timer callback */
            if (timer->callback)
            {
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
                        Timer_insert(timer);
                        break;
                }
                /* restart loop from the beginning in case list has changed */
                current = list;
            }
            WDT_kick();
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
void Timer_init(uint16_t instance, Timer_Config *config)
{
    if (registers != NULL)
    {
        /* already initialized */
        return;
    }

    registers = (volatile Timer_A_Registers*)instance;

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
