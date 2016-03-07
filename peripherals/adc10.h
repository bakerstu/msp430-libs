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
 * @file adc10.h
 * 10-bit ADC methods.
 *
 * @author Stuart W. Baker
 * @date 28 April 2015
 */

#ifndef ADC10_H_
#define ADC10_H_

#include <msp430.h>
#include <stdint.h>

/** Channel inputs supported by the ADC.
 */
typedef enum
{
    ADC_CH_0      = 0,  ///< ADC input channel 0
    ADC_CH_1      = 1,  ///< ADC input channel 1
    ADC_CH_2      = 2,  ///< ADC input channel 2
    ADC_CH_3      = 3,  ///< ADC input channel 3
    ADC_CH_4      = 4,  ///< ADC input channel 4
    ADC_CH_5      = 5,  ///< ADC input channel 5
    ADC_CH_6      = 6,  ///< ADC input channel 6
    ADC_CH_7      = 7,  ///< ADC input channel 7
    ADC_CH_8      = 8,  ///< ADC input channel 8
    ADC_CH_9      = 9,  ///< ADC input channel 9
    ADC_CH_10     = 10, ///< ADC input channel 10
    ADC_CH_11     = 11, ///< ADC input channel 11
    ADC_CH_12     = 12, ///< ADC input channel 12
    ADC_CH_13     = 13, ///< ADC input channel 13
    ADC_CH_14     = 14, ///< ADC input channel 14
    ADC_CH_15     = 15, ///< ADC input channel 15
    ADC_VREF_P    = 8, ///< ADC input channel VREF+
    ADC_VREF_N    = 9, ///< ADC input channel VREF-
    ADC_TEMP      = 10, ///< ADC input channel temperature
    ADC_VCC_DIV_2 = 11, ///< ADC input channel (VCC - VSS) / 2

} ADC_Channel;

/** Initialize the ADC module, including perform a calibration.
 */
void ADC_init(void);

/** Perform a (re-)calibration.
 */
void ADC_calibrate(void);

/** Convert a single ADC channel.
 * @param channel number to convert
 * @return 10-bit converted value with calibration offsets factored in
 */
uint16_t ADC_convert(ADC_Channel channel);

#endif /* ADC10_H_ */
