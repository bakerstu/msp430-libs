/**
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

#endif /* INCLUDE_EEPROM_EMULATION_H_ */
