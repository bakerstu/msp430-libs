/**
 * @file eeprom.h EEPROM access methods.
 *
 * @author Stuart W. Baker
 * @date 6 January 2016
 */

#include <stdint.h>

#ifndef INCLUDE_EEPROM_H_
#define INCLUDE_EEPROM_H_

/** number of bytes the EEPROM can hold */
#define EEPROM_BYTE_COUNT 128

/** Initialize EEPROM.
 */
void EEPROM_init();

/** Write a byte of data to the EEPROM.
 * @param index EEPROM address index to write
 */
void EEPROM_write(uint8_t index, uint8_t data);

/** Read a byte of data out of the EEPROM.
 * @param index EEPROM address index to read
 * @return data contained at EEPROM address index
 */
uint8_t EEPROM_read(uint8_t index);

#endif /* INCLUDE_EEPROM_H_ */
