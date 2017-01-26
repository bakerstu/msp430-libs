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
 * @file adc10.c
 * 10-bit ADC methods.
 *
 * @author Stuart W. Baker
 * @date 28 April 2015
 */

#include "adc10.h"
#include "lpm.h"

static int16_t adcOffsetError;

static void ADC_enable(ADC_Channel channel)
{
#if !defined (__MSP430G2553__)
    if (channel >= 8)
    {
        ADC10AE1 |= 0x01 << channel;
    }
    else
#endif
    {
        ADC10AE0 |= 0x01 << channel;
    }
}

/** Initialize the ADC module, including perform a calibration.
 */
void ADC_init(void)
{
    ADC10CTL1 = ADC10DIV_7;
    ADC_calibrate();
}

/** Perform a (re-)calibration.
 */
void ADC_calibrate(void)
{
    ADC10CTL0 = ADC10SHT_3 + ADC10ON + ADC10IE;
    ADC10CTL1 = ADC10DIV_7 + ADC_VCC_DIV_2 * INCH_1;
    ADC10CTL0 |= ENC + ADC10SC;
    LPM_enter0();
    ADC10CTL0 = 0;

    adcOffsetError = 511 - ADC10MEM;

    ADC10CTL0 &= ~ENC;
}


/** Convert a single ADC channel.
 * @param channel number to convert
 * @return 10-bit converted value with calibration offsets factored in
 */
uint16_t ADC_convert(ADC_Channel channel)
{
    int16_t result;

    ADC_enable(channel);

    ADC10CTL0 = ADC10SHT_3 + ADC10ON + ADC10IE;
    ADC10CTL1 = ADC10DIV_7 + channel * INCH_1;
    ADC10CTL0 |= ENC + ADC10SC;
    LPM_enter0();
    ADC10CTL0 = 0;

    result = ADC10MEM + adcOffsetError;

    if (result < 0)
    {
        result = 0;
    }
    if (result > 1023)
    {
        result = 1024;
    }

    return result;
}

/** ADC10 interrupt service routine.
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC10_VECTOR))) ADC10_ISR (void)
#else
#error Compiler not supported!
#endif
{
    LPM_exit0();
}
