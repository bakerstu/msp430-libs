/**
 * @file low_power_mode.h
 * Manage low power modes.
 *
 * @author Stuart W. Baker
 * @date 20 April 2015
 */

#ifndef LPM_MODE_H_
#define LPM_MODE_H_

#include <msp430.h>

#include <stdint.h>

/** Enumeration of all the supported LPM modes.
 */
typedef enum
{
    LPM_MODE_NONE, ///< not currently in an LPM state
    LPM_MODE_0,     ///< low power mode 0
    LPM_MODE_1,     ///< low power mode 2
    LPM_MODE_2,     ///< low power mode 2
    LPM_MODE_3,     ///< low power mode 3
    LPM_MODE_4,     ///< low power mode 4
} LPM_Mode;

/** Keeps track of current LPM mode.  This is a private variable and should
 * not be used directly by the application.
 */
extern LPM_Mode lpmMode;

/** Enter low power mode 0.  Should not be called from outside ISR context.
 */
inline void LPM_enter0(void)
{
    __disable_interrupt();
    lpmMode = LPM_MODE_0;
    __bis_SR_register(LPM0_bits + GIE);
}

/** Enter low power mode 1.  Should not be called from outside ISR context.
 */
inline void LPM_enter1(void)
{
    __disable_interrupt();
    lpmMode = LPM_MODE_1;
    __bis_SR_register(LPM1_bits + GIE);
}

/** Enter low power mode 2.  Should not be called from outside ISR context.
 */
inline void LPM_enter2(void)
{
    __disable_interrupt();
    lpmMode = LPM_MODE_2;
    __bis_SR_register(LPM2_bits + GIE);
}

/** Enter low power mode 3.  Should not be called from outside ISR context.
 */
inline void LPM_enter3(void)
{
    __disable_interrupt();
    lpmMode = LPM_MODE_3;
    __bis_SR_register(LPM3_bits + GIE);
}

/** Enter low power mode 4.  Should not be called from outside ISR context.
 */
inline void LPM_enter4(void)
{
    __disable_interrupt();
    lpmMode = LPM_MODE_4;
    __bis_SR_register(LPM4_bits + GIE);
}

/** Exit low power mode 0.  Should only be called from ISR context.
 */
#define LPM_exit0() \
    lpmMode = LPM_MODE_NONE; \
    LPM0_EXIT

/** Exit low power mode 1.  Should only be called from ISR context.
 */
#define LPM_exit1() \
    lpmMode = LPM_MODE_NONE; \
    LPM1_EXIT

/** Exit low power mode 2.  Should only be called from ISR context.
 */
#define LPM_exit2() \
    lpmMode = LPM_MODE_NONE; \
    LPM2_EXIT

/** Exit low power mode 3.  Should only be called from ISR context.
 */
#define LPM_exit3() \
    lpmMode = LPM_MODE_NONE; \
    LPM3_EXIT

/** Exit low power mode 4.  Should only be called from ISR context.
 */
#define LPM_exit4() \
    lpmMode = LPM_MODE_NONE; \
    LPM4_EXIT

/** Exit current low power mode.  Should only be called from ISR context.
 */
#define LPM_exit()           \
do                           \
{                            \
    LPM_Mode mode = lpmMode; \
    lpmMode = LPM_MODE_NONE; \
                             \
    switch (mode)            \
    {                        \
        default:             \
        case LPM_MODE_NONE:  \
            break;           \
        case LPM_MODE_0:     \
            LPM0_EXIT        \
            break;           \
        case LPM_MODE_1:     \
            LPM1_EXIT        \
            break;           \
        case LPM_MODE_2:     \
            LPM2_EXIT        \
            break;           \
        case LPM_MODE_3:     \
            LPM3_EXIT        \
            break;           \
        case LPM_MODE_4:     \
            LPM4_EXIT        \
            break;           \
    }                        \
} while (0);

/** Get the current low power mode.
 * @return current low power mode
 */
inline LPM_Mode LPM_getMode(void)
{
    return lpmMode;
}

#endif /* LPM_MODE_H_ */
