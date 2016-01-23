/**
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
    ADC10CTL0 = ADC10SHT_3 + /*ADC10SR*/ + ADC10ON + ADC10IE;
    ADC10CTL1 = ADC10DIV_7;


    ADC_calibrate();
}

/** Perform a (re-)calibration.
 */
void ADC_calibrate(void)
{
    ADC10CTL1 = ADC10DIV_7 + ADC_VCC_DIV_2 * INCH_1;
    ADC10CTL0 |= ENC + ADC10SC;
    LPM_enter0();

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

    ADC10CTL1 = ADC10DIV_7 + channel * INCH_1;
    ADC10CTL0 |= ENC + ADC10SC;
    LPM_enter0();

    result = ADC10MEM + adcOffsetError;

    ADC10CTL0 &= ~ENC;

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
