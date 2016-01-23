/**
 * @file eeprom_emulation.c EEPROM emulation in FLASH.
 *
 * @author Stuart W. Baker
 * @date 23 October 2015
 */

#include "eeprom_emulation.h"

#include <msp430.h>

#include "wdt.h"

extern uint16_t eepromMem[FLASH_SIZE/sizeof(uint16_t)];

uint8_t shaddowRam[EEPROM_BYTE_COUNT];

static int activeIndex = 0;
static int available = 0;

/** Return the number of sectors being used for emulation.
 * @return sector count
 */
static int sectorCount(void)
{
    return FLASH_SIZE / SECTOR_SIZE;
}

/** Return the start of a given sector index.
 * @param index sector index
 * @return start address of sector index
 */
static uint16_t *sector(int index)
{
    return eepromMem + (index * (SECTOR_SIZE / BLOCK_SIZE));
}

/** Return the start of a given block within a given sector start address.
 * @param index block index within the given sector
 * @param sector_addr start address of the sector
 */
static uint16_t *block(int index, uint16_t *sector_addr)
{
    return sector_addr + index;
}

/** Get the sector index.
 * @param sector_address absolute sector address to get index from
 * @return index of sector relative to the start of the EEPROM region
 */
static int sectorIndex(uint16_t *sector_address)
{
    return ((uintptr_t)sector_address - (uintptr_t)eepromMem) / SECTOR_SIZE;
}

/** Return the start address of the currently active sector.
 * @return start address of the currently active sector
 */
static uint16_t *active(void)
{
    return sector(activeIndex);
}

/** Get the next active sector pointer.
 * @return a pointer to the beginning of the next active sector
 */
static uint16_t *nextActive(void)
{
    return ((activeIndex + 1) < sectorCount()) ? sector(activeIndex + 1) :
                                                  sector(0);
}

/** Erase sector.
 * @param sector_addr start address of the sector to erase
 */
static void flashErase(uint16_t* sector_addr)
{
    uint16_t context = WDT_extend();
    /* SMCLK = 2MHz, divide = 6 = (5 + 1), fFTG = 1MHz / 3 = 333.33 kHz */
    FCTL2 = FWKEY + FSSEL_2 + 5;
    FCTL3 = FWKEY;
    FCTL1 = FWKEY + ERASE;
    *sector_addr = 0;
    FCTL3 = FWKEY + LOCK;
    WDT_restore(context);
}

/** flash sector.
 * @param data data value to flash
 * @param address location to flash
 */
static void flashProgram(uint16_t data, uint16_t* address)
{
    uint16_t context = WDT_extend();
    /* SMCLK = 1MHz, divide = 6 = (5 + 1), fFTG = 1MHz / 3 = 333.33 kHz */
    FCTL2 = FWKEY + FSSEL_2 + 5;
    FCTL3 = FWKEY;
    FCTL1 = FWKEY + WRT;
    *address = data;
    FCTL1 = FWKEY;
    FCTL3 = FWKEY + LOCK;
    WDT_restore(context);
}

/** Total number of EEPROM slots in a FLASH sector.  A slot is the same as a
 * block, except that it excludes any metadata blocks.
 * @return number of EEPROM slots in a FLASH block
 */
static int slotCount(void)
{
    return (SECTOR_SIZE / BLOCK_SIZE) - MAGIC_COUNT;
}

/** Slot data pointer pointing to the last slot in a given sector
 * as a block, except that it excludes any metadata blocks.
 * @param sector_address pointer to the beginning of the sector
 * @return address pointer to the last slot in the sector
 */
static uint16_t *slotLast(uint16_t *sector_address)
{
    return sector_address + (MAGIC_COUNT + slotCount() - 1 );
}

/** Block data pointer pointing to the last magic block in a given sector.
 * @param sector_address pointer to the beginning of the sector
 * @return address pointer to the last magic block in the sector
 */
static uint16_t *magicLast(uint16_t *sector_address)
{
    return sector_address + (MAGIC_COUNT - 1);
}

/** Initialize EEPROM.
 */
void EEPROM_init()
{
    for (int i = 0; i < sectorCount(); ++i)
    {
        if (*block(MAGIC_DIRTY_INDEX,  sector(i)) == MAGIC_DIRTY  &&
            *block(MAGIC_INTACT_INDEX, sector(i)) == MAGIC_INTACT &&
            *block(MAGIC_USED_INDEX,   sector(i)) == MAGIC_ERASED)
        {
            activeIndex = i;
            break;
        }
    }

    if (*block(MAGIC_DIRTY_INDEX,  active()) != MAGIC_DIRTY  ||
        *block(MAGIC_INTACT_INDEX, active()) != MAGIC_INTACT ||
        *block(MAGIC_USED_INDEX,   active()) != MAGIC_ERASED)
    {
        /* our active block is corrupted, we are starting over */
        flashErase(active());
        flashProgram(MAGIC_DIRTY, block(MAGIC_DIRTY_INDEX, active()));
        flashProgram(MAGIC_INTACT, block(MAGIC_INTACT_INDEX, active()));
        available = slotCount();
    }
    else
    {
        /* look for first data block */
        for (uint16_t *address = slotLast(active());
             address != magicLast(active());
             --address, ++available)
        {
            if (*address != MAGIC_ERASED)
            {
                break;
            }
        }
    }

    for (unsigned int i = 0; i < EEPROM_BYTE_COUNT; ++i)
    {
        shaddowRam[i] = 0xFF; // default value if not found
        for (uint16_t *address = slotLast(active());
             address != magicLast(active());
             --address)
        {
            if (i == (*address >> 8))
            {
                /* found the data */
                shaddowRam[i] = *address & 0xFF;
                break;
            }
        }

    }
}

/** Write a byte of data to the EEPROM.
 * @param index EEPROM address index to write
 */
void EEPROM_write(uint8_t index, uint8_t data)
{
    if (index >= EEPROM_BYTE_COUNT)
    {
        /* we only support up to index EEPROM_BYTE_COUNT - 1 */
        return;
    }

    if (EEPROM_read(index) != data)
    {
        /* new data value to store */
        shaddowRam[index] = data;

        if (available)
        {
            uint16_t *address = block(MAGIC_COUNT + slotCount() - available,
                                      active());
            flashProgram((uint16_t)index << 8 | (uint16_t)data, address);
            --available;
        }
        else
        {
            /* we need to overflow into the next block */
            uint16_t *next_sector = nextActive();

            flashErase(next_sector);
            flashProgram(MAGIC_DIRTY, block(MAGIC_DIRTY_INDEX, next_sector));

            /* reset the available count */
            available = slotCount();

            uint16_t *address = next_sector + MAGIC_COUNT;

            /* move any existing data over */
            for (unsigned int i = 0; i < EEPROM_BYTE_COUNT; ++i)
            {
                uint16_t slot_data = i << 8;
                if (i == index)
                {
                    slot_data += data;
                }
                else
                {
                    slot_data += EEPROM_read(i);
                }
                if ((slot_data & 0xFF) != 0xFF)
                {
                    flashProgram(slot_data, address);
                    ++address;
                    --available;
                }
            }
            /* finalize the data move and write */
            flashProgram(MAGIC_INTACT, block(MAGIC_INTACT_INDEX, next_sector));
            flashProgram(MAGIC_USED, block(MAGIC_USED_INDEX, active()));
            activeIndex = sectorIndex(next_sector);
        }
    }
}

/** Read a byte of data out of the EEPROM.
 * @param index EEPROM address index to read
 * @return data contained at EEPROM address index
 */
uint8_t EEPROM_read(uint8_t index)
{
    if (index >= EEPROM_BYTE_COUNT)
    {
        /* we only support up to index EEPROM_BYTE_COUNT - 1 */
        return 0xFF;
    }

    /* we don't necessarily store data when it is equal 0xFF, so assume value
     * is 0xFF if not found.
     */
    return shaddowRam[index];
}
