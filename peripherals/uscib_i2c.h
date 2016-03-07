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

#ifndef USCIB_I2C_H_
#define USCIB_I2C_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/** Base address of USCI_B0 Registers */
#define USCI_B0_MODULE ((uint16_t)&UCB0CTL0)

/** Base address of USCI_B1 Registers */
#define USCI_B1_MODULE ((uint16_t)&UCB1CTL0)

/** USCI B ACLK clock source */
#define USCI_B_I2C_CLOCK_SOURCE_ACLK (UCSSEL_1)

/** USCI B SMCLK clock source */
#define USCI_B_I2C_CLOCK_SOURCE_SMCLK (UCSSEL_2)

/** USCI B 400 Kbps data rate */
#define USCI_B_I2C_SET_DATA_RATE_400KBPS (400000)

/** USCI B 100 Kbps data rate */
#define USCI_B_I2C_SET_DATA_RATE_100KBPS (100000)

/** Configuration parameters for configuring the I2C master.
 */
typedef struct
{
    /** Source of I2C clock.  Valid values are
     * - \b USCI_B_I2C_CLOCK_SOURCE_ACLK
     * - \b USCI_B_I2C_CLOCK_SOURCE_SMCLK
     */
    uint8_t selectClockSource;
    /** Rate in Hz of the clock source specified in selectClockSource. */
    uint32_t i2cClk;
    /** Set up for selecting data transfer rate.  Valid values are
     * - \b USCI_B_I2C_SET_DATA_RATE_400KBPS
     * - \b USCI_B_I2C_SET_DATA_RATE_100KBPS
     */
    uint32_t dataRate;
} USCI_I2C_MasterConfig;

/** Initialize USCI B as an I2C master.
 *
 * @param instance Instance the instance of the USCI B (I2C) module.
 *                       Valid values include:
 *                        - \b USCI_B0_MODULE
 *                        - \b USCI_B1_MODULE
 * @param config Confugration structure for the I2C master mode.
 */
void I2C_initMaster(uint16_t instance, USCI_I2C_MasterConfig *config);

/** Send then receive an I2C data.
 *
 * @param instance Instance the instance of the USCI B (I2C) module.
 *                       Valid values include:
 *                        - \b USCI_B0_MODULE
 *                        - \b USCI_B1_MODULE
 * @param address I2C device address
 * @param sendData pointer to data for sending
 * @param sendSize size in number of bytes of data to send
 * @param receiveData pointer to data for reception
 * @param receiveSize size in number of bytes of data to receive
 * @return true upon success, false on error
 */
bool I2C_sendReceive(uint16_t instance, uint8_t address,
                     void *sendData, size_t sendSize,
                     void *receiveData, size_t receiveSize);

/**
 * Receive an I2C packet.
 *
 * @param instance Instance the instance of the USCI B (I2C) module.
 *                       Valid values include:
 *                        - \b USCI_B0_MODULE
 *                        - \b USCI_B1_MODULE
 * @param address I2C device address
 * @param data pointer to data for reception
 * @param size size in number of bytes of data to receive
 * @return true upon success, false on error
 */
bool I2C_receive(uint16_t instance, uint8_t address, void *data, size_t size);

/**
 * Transmit an I2C packet.
 *
 * @param instance Instance the instance of the USCI B (I2C) module.
 *                       Valid values include:
 *                        - \b USCI_B0_MODULE
 *                        - \b USCI_B1_MODULE
 * @param address I2C device address
 * @param data pointer to data for transmission
 * @param size size in number of bytes of data to transmit
 * @return true upon success, false on error
 */
bool I2C_send(uint16_t instance, uint8_t address, const void *data, size_t size);

#endif /* USCIB_I2C_H_ */
