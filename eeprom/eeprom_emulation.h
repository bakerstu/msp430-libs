/**
 * @copyright
 * Copyright (c) 2015, Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 *
 * Texas Instruments (TI) is supplying this software for use solely and
 * exclusively on TI's microcontroller products. The software is owned by
 * TI and/or its suppliers, and is protected under applicable copyright
 * laws. You may not combine this software with "viral" open-source
 * software in order to form a larger program.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
 * NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
 * NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
 * CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * @file eeprom_emulation.h EEPROM emulation in FLASH.
 *
 * @author Stuart W. Baker
 * @date 7 January 2015
 */

#ifndef INCLUDE_EEPROM_EMULATION_H_
#define INCLUDE_EEPROM_EMULATION_H_

#include "eeprom.h"

/** size of writable block in bytes */
#define BLOCK_SIZE 2

/** Sector size of the FLASH being used for EEPROM emulation */
#define SECTOR_SIZE 512

/** Size of flash region to be used for EEPROM emulation */
#define FLASH_SIZE (SECTOR_SIZE << 1)

#define MAGIC_DIRTY  0xaa55
#define MAGIC_INTACT 0x8001
#define MAGIC_USED   0x0000
#define MAGIC_ERASED 0xFFFF

#define MAGIC_DIRTY_INDEX  0
#define MAGIC_INTACT_INDEX 1
#define MAGIC_USED_INDEX   2
#define MAGIC_COUNT        3

#define MAGIC_CONST      0xFF00
#define MAGIC_INDEX      0xFE00
#define MAGIC_CONST_MASK 0xFF00

#endif /* INCLUDE_EEPROM_EMULATION_H_ */
