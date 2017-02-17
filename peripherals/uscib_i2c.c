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
 * @file uscib_i2c.h I2C Master driver using USCIB0 peripheral.
 *
 * @author Stuart W. Baker
 * @date 19 February 2015
 */

#include <msp430.h>

#include "msp430_libs_user.h"

#include "uscib_i2c.h"
#include "lpm.h"
#include "wdt.h"
#include "timer.h"
#include "gpio.h"

#if defined (UC1IE)
/** number of channels this driver supports */
#define CHANNEL_COUNT 2
#else
/** number of channels this driver supports */
#define CHANNEL_COUNT 1

#endif

/** Status of I2C transaction.
 */
typedef enum
{
    I2C_SUCCESS = 0, /**< transaction completed successfully */
    I2C_PENDING,     /**< transaction pending */
    I2C_FAIL,        /**< transaction failure */
} USCIB_I2C_Fault;

/** Transaction size */
static size_t i2cSize[CHANNEL_COUNT];

/** Transaction data pointer */
static uint8_t *i2cData[CHANNEL_COUNT];

/** Transaction fault flag */
static USCIB_I2C_Fault i2cFault[CHANNEL_COUNT];

/** Transaction direction flag */
static bool i2cTransmit[CHANNEL_COUNT];

/** USCI I2C registers.
 */
typedef struct
{
    volatile uint8_t UCBxCTL0; /**< control register 0 */
    volatile uint8_t UCBxCTL1; /**< control register 1 */
    union
    {
        struct
        {
            volatile uint8_t UCBxBR0; /**< bit rate control register 0 */
            volatile uint8_t UCBxBR1; /**< bit rate control register 1 */
        };
        volatile uint16_t UCBxBR; /**< bit rate control register */
    };
    volatile uint8_t UCBxI2CIE; /**< interrupt enable register */
    volatile uint8_t UCBxSTAT; /**< status gregister */
    volatile uint8_t UCBxRXBUF; /**< receive buffer register */
    volatile uint8_t UCBxTXBUF; /**< transmit buffer register */
} USCI_B_I2C_Registers;

/** USCI I2C registers.
 */
typedef struct
{
    volatile uint8_t UCBxI2COA; /**< own address register */
    volatile uint8_t pad; /**< address padding */
    volatile uint8_t UCBxI2CSA; /**< slave address register */
} USCI_B_I2C_Address_Registers;

#define USCI_B0_ADDRESS_REGISTER_BASE \
    ((volatile USCI_B_I2C_Address_Registers*)&UCB0I2COA)
#define USCI_B0_IE IE2
#define USCI_B0_IFG IFG2
#define USCI_B0_TXIE UCB0TXIE
#define USCI_B0_RXIE UCB0RXIE
#define USCI_B0_TXIFG UCB0TXIFG
#define USCI_B0_RXIFG UCB0RXIFG

#if defined (UC1IE)
#define USCI_B1_ADDRESS_REGISTER_BASE \
    ((volatile USCI_B_I2C_Address_Registers*)&UCB1I2COA)
#define USCI_B1_IE UC1IE
#define USCI_B1_IFG UC1IFG
#define USCI_B1_TXIE UCB1TXIE
#define USCI_B1_RXIE UCB1RXIE
#define USCI_B1_TXIFG UCB1TXIFG
#define USCI_B1_RXIFG UCB1RXIFG
#endif

/*
 * I2C_initMaster
 */
void I2C_initMaster(uint16_t instance, USCI_I2C_MasterConfig *config)
{
    volatile USCI_B_I2C_Registers *registers =
        (volatile USCI_B_I2C_Registers*)instance;

    /* Disable RX and TX interrupts */
    if (instance == USCI_B0_MODULE)
    {
        USCI_B0_IE &= ~(USCI_B0_TXIE | USCI_B0_RXIE);
    }
#if defined (UC1IE)
    else if (instance == USCI_B1_MODULE)
    {
        USCI_B1_IE &= ~(USCI_B1_TXIE | USCI_B1_RXIE);
    }
#endif
    else
    {
        /* invalid instance */
        return;
    }

    /* Enable SW reset */
    registers->UCBxCTL1 |= UCSWRST;

    /* I2C Master, synchronous mode */
    registers->UCBxCTL0 = UCMST + UCMODE_3 + UCSYNC;

    /* Clock setup, stay in reset */
    registers->UCBxCTL1 = config->selectClockSource;
    registers->UCBxBR = (config->i2cClk / config->dataRate);

    /* Clear SW reset, resume operation */
    registers->UCBxCTL1 &= ~UCSWRST;
}

/*
 * I2C_sendReceive
 */
bool I2C_sendReceive(uint16_t instance, uint8_t address,
                     void *sendData, size_t sendSize,
                     void *receiveData, size_t receiveSize)
{
    if (I2C_send(instance, address, sendData, sendSize) == false)
    {
        return false;
    }
    return I2C_receive(instance, address, receiveData, receiveSize);
}

/*
 * I2C_receive
 */
bool I2C_receive(uint16_t instance, uint8_t address, void *data, size_t size)
{
    volatile USCI_B_I2C_Registers *registers =
        (volatile USCI_B_I2C_Registers*)instance;
    volatile USCI_B_I2C_Address_Registers *addressRegisters;
    int index;

    if (size == 0)
    {
        return false;
    }

    /* Disable TX interrupt, setup instance index, and setup addressRegisters
     * pointer
     */
    if (instance == USCI_B0_MODULE)
    {
        USCI_B0_IE &= ~(USCI_B0_TXIE);
        index = 0;
        addressRegisters = USCI_B0_ADDRESS_REGISTER_BASE;
    }
#if defined (UC1IE)
    else if (instance == USCI_B1_MODULE)
    {
        USCI_B1_IE &= ~(USCI_B1_TXIE);
        index = 1;
        addressRegisters = USCI_B1_ADDRESS_REGISTER_BASE;
    }
#endif
    else
    {
        /* invalid instance */
        return false;
    }

    /* transaction size */
    i2cSize[index] = size - 1;

    /* store data pointer for transaction */
    i2cData[index] = (uint8_t*)data;

    /* assume a fault until successful */
    i2cFault[index] = I2C_PENDING;

    /* setup transaction as a receive */
    i2cTransmit[index] = false;

    /* setup the slave address */
    addressRegisters->UCBxI2CSA = address;

    /* Ensure stop condition got sent */
    while (registers->UCBxCTL1 & UCTXSTP);

    /* I2C RX */
    registers->UCBxCTL1 &= ~UCTR;

    /* I2C start condition */
    registers->UCBxCTL1 |= UCTXSTT;

    /* if this is a single byte transaction */
    if (i2cSize[index] == 0)
    {
        /* wait for start condition sent */
        while (registers->UCBxCTL1 & UCTXSTT);

        /* stop condition after first read byte */
        registers->UCBxCTL1 |= UCTXSTP;
    }

    I2C_watchdogKick();

    /* Enable RX interrupt.
     */
    if (instance == USCI_B0_MODULE)
    {
        USCI_B0_IE |= USCI_B0_RXIE;
    }
#if defined (UC1IE)
    else if (instance == USCI_B1_MODULE)
    {
        USCI_B1_IE |= USCI_B1_RXIE;
    }
#endif
    else
    {
        /* invalid instance */
        return false;
    }

    /* Enter LPM0 w/interrupts */
    //LPM_enter0();
    uint64_t start_time = Timer_gettime();
    do
    {
        I2C_watchdogKick();
        Timer_spinDelay(1);
        if (i2cFault[index] == I2C_SUCCESS)
        {
            break;
        }
    } while (Timer_gettime() < (start_time + (size / 5) + 1));

    return (i2cFault[index] != I2C_SUCCESS) ? false : true;
}

/*
 * I2C_send
 */
bool I2C_send(uint16_t instance, uint8_t address, const void *data, size_t size)
{
    volatile USCI_B_I2C_Registers *registers =
        (volatile USCI_B_I2C_Registers*)instance;
    volatile USCI_B_I2C_Address_Registers *addressRegisters;
    int index;

    if (size == 0)
    {
        return false;
    }

    /* Disable RX interrupt, setup instance index, and setup addressRegisters
     * pointer
     */
    if (instance == USCI_B0_MODULE)
    {
        USCI_B0_IE &= ~(USCI_B0_RXIE);
        index = 0;
        addressRegisters = USCI_B0_ADDRESS_REGISTER_BASE;
    }
#if defined (UC1IE)
    else if (instance == USCI_B1_MODULE)
    {
        USCI_B1_IE &= ~(USCI_B1_RXIE);
        index = 1;
        addressRegisters = USCI_B1_ADDRESS_REGISTER_BASE;
    }
#endif
    else
    {
        /* invalid instance */
        return false;
    }

    /* transaction size */
    i2cSize[index] = size;

    /* store data pointer for transaction */
    i2cData[index] = (uint8_t*)data;

    /* assume a fault until successful */
    i2cFault[index] = I2C_PENDING;

    /* setup transaction as a receive */
    i2cTransmit[index] = true;

    /* setup the slave address */
    addressRegisters->UCBxI2CSA = address;

    /* Ensure stop condition got sent */
    while (registers->UCBxCTL1 & UCTXSTP);

    /* I2C TX, start condition */
    registers->UCBxCTL1 |= UCTR + UCTXSTT;

    I2C_watchdogKick();

    /* Enable TX interrupt.
     */
    if (instance == USCI_B0_MODULE)
    {
        USCI_B0_IE |= USCI_B0_TXIE;
    }
#if defined (UC1IE)
    else if (instance == USCI_B1_MODULE)
    {
        USCI_B1_IE |= USCI_B1_TXIE;
    }
#endif
    else
    {
        /* invalid instance */
        return false;
    }

    /* Enter LPM0 w/interrupts */
    //LPM_enter0();
    uint64_t start_time = Timer_gettime();
    do
    {
        I2C_watchdogKick();
        Timer_spinDelay(1);
        if (i2cFault[index] == I2C_SUCCESS)
        {
            break;
        }
    } while (Timer_gettime() < (start_time + (size / 5) + 1));

    return (i2cFault[index] != I2C_SUCCESS) ? false : true;
}

/**
 * The USCIAB0TX ISR is used to move received data from the I2C slave
 * to the MSP430 memory or transmit data from the MSP430 memory to the I2C
 * slave.  For the most part, this ISR will run after waking up from low power
 * mode.  On return, the MSP430 returns to low power mode, unless the
 * current transaction is complete, or results in an error.
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCIAB0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    volatile USCI_B_I2C_Registers *registers =
        (volatile USCI_B_I2C_Registers*)USCI_B0_MODULE;

    if(i2cTransmit[0])
    {
        /* This is a transmit transaction */
        if (i2cSize[0])
        {
            /* bytes left to send */
            registers->UCBxTXBUF = *i2cData[0]++;
            --i2cSize[0];
        }
        else
        {
            /* no more bytes to send, send stop condition, exit LPM0 */
            registers->UCBxCTL1 |= UCTXSTP;
            USCI_B0_IFG &= ~(USCI_B0_TXIFG);
            i2cFault[0] = I2C_SUCCESS;
            //LPM_exit0();
        }
    }
    else
    {
        /* receive last byte, send stop condition, exit LPM0 */
        *i2cData[0]++ = registers->UCBxRXBUF;
        switch (i2cSize[0])
        {
            case 1:
                registers->UCBxCTL1 |= UCTXSTP;
                /* fall through */
            default:
                --i2cSize[0];
                break;
            case 0:
                i2cFault[0] = I2C_SUCCESS;
                //LPM_exit0();
                break;
        }
    }
}
