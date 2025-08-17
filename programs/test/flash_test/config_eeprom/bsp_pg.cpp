// Copyright (c) 2025. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by Administrator on 2025/7/28.
//

#include "bsp_pg.h"

#include <crc8.h>
#include <string.h>

#include "bsp_print.h"
#include "crc_check.h"

#define MIN(a, b)               \
    __extension__({             \
        __typeof__(a) _a = (a); \
        __typeof__(b) _b = (b); \
        _a < _b ? _a : _b;      \
    })
#define MAX(a, b)               \
    __extension__({             \
        __typeof__(a) _a = (a); \
        __typeof__(b) _b = (b); \
        _a > _b ? _a : _b;      \
    })
#define ABS(x)                  \
    __extension__({             \
        __typeof__(x) _x = (x); \
        _x > 0 ? _x : -_x;      \
    })

static uint16_t eepromConfigSize;

// Header for the saved copy.
typedef struct {
    uint8_t eepromConfigVersion;
    uint8_t magic_be;  // magic number, should be 0xBE
} PG_PACKED configHeader_t;

// Header for each stored PG.
typedef struct {
    // split up.
    uint16_t size;
    pgn_t pgn;
    uint8_t version;
    // lower 2 bits used to indicate system or profile number, see CR_CLASSIFICATION_MASK
    uint8_t flags;
    uint8_t pg[];
} PG_PACKED configRecord_t;

// Footer for the saved copy.
typedef struct {
    uint16_t terminator;
} PG_PACKED configFooter_t;
// checksum is appended just after footer. It is not included in footer to make checksum calculation
// consistent

// Used to check the compiler packing at build time.
typedef struct {
    uint8_t byte;
    uint32_t word;
} PG_PACKED packingTest_t;

const pgRegistry_t* pgFind(pgn_t pgn) {
    if (!__pg_registry_start || !__pg_registry_end)
        return NULL;

    PG_FOREACH(reg) {
        if (pgN(reg) == pgn) {
            return reg;
        }
    }
    return NULL;
}

static uint8_t* pgOffset(const pgRegistry_t* reg) {
    return reg->address;
}

void pgResetInstance(const pgRegistry_t* reg, uint8_t* base) {
    const uint16_t regSize = pgSize(reg);

    memset(base, 0, regSize);
    if (reg->reset.ptr) {
        if (__pg_resetdata_start && __pg_resetdata_end &&
            reg->reset.ptr >= (void*)__pg_resetdata_start &&
            reg->reset.ptr < (void*)__pg_resetdata_end) {
            // pointer points to resetdata section, to it is data template
            memcpy(base, reg->reset.ptr, regSize);
        } else if (reg->reset.fn) {
            // reset function, call it
            reg->reset.fn(base, regSize);
        }
    }
}

void pgReset(const pgRegistry_t* reg) {
    pgResetInstance(reg, pgOffset(reg));
}

bool pgResetCopy(uint8_t* copy, pgn_t pgn) {
    const pgRegistry_t* reg = pgFind(pgn);
    if (reg) {
        pgResetInstance(reg, copy);
        return true;
    }
    return false;
}

bool pgLoad(const pgRegistry_t* reg, const void* from, int size, int version) {
    pgResetInstance(reg, pgOffset(reg));
    // restore only matching version, keep defaults otherwise
    if (version == pgVersion(reg)) {
        const int take = MIN(size, pgSize(reg));
        memcpy(pgOffset(reg), from, take);

        return true;
    }
    return false;
}

int pgStore(const pgRegistry_t* reg, void* to, int size) {
    const int take = MIN(size, pgSize(reg));
    memcpy(to, pgOffset(reg), take);
    return take;
}

void pgResetAll(void) {
    if (!__pg_registry_start || !__pg_registry_end)
        return;

    PG_FOREACH(reg) {
        pgReset(reg);
    }
}

/**
 * @brief Initialize the EEPROM.
 */

void initEEPROM(void) {
    // Verify that this architecture packs as expected.
    BUILD_BUG_ON(offsetof(packingTest_t, byte) != 0);
    BUILD_BUG_ON(offsetof(packingTest_t, word) != 1);
    BUILD_BUG_ON(sizeof(packingTest_t) != 5);

    BUILD_BUG_ON(sizeof(configFooter_t) != 2);
    BUILD_BUG_ON(sizeof(configRecord_t) != 6);
}

uint16_t getEEPROMConfigSize(void) {
    return eepromConfigSize;
}

// find config record for reg + classification (profile info) in EEPROM
// return NULL when record is not found
// this function assumes that EEPROM content is valid
const configRecord_t* findEEPROM(const pgRegistry_t* reg, configRecordFlags_e classification) {
    const uint8_t* p = __config_start;
    p += sizeof(configHeader_t);  // skip header
    while (true) {
        const configRecord_t* record = (const configRecord_t*)p;
        if (record->size == 0 || p + record->size >= __config_end || record->size < sizeof(*record))
            break;
        if (pgN(reg) == record->pgn && (record->flags & CR_CLASSIFICATION_MASK) == classification)
            return record;
        p += record->size;
    }
    // record not found
    return NULL;
}

// Initialize all PG records from EEPROM.
// This functions processes all PGs sequentially, scanning EEPROM for each one. This is suboptimal,
//   but each PG is loaded/initialized exactly once and in defined order.
bool loadEEPROM(void) {
    bool success = true;

    PG_FOREACH(reg) {
        const configRecord_t* rec = findEEPROM(reg, CR_CLASSICATION_SYSTEM);
        if (rec) {
            // config from EEPROM is available, use it to initialize PG. pgLoad will handle version
            // mismatch
            if (!pgLoad(reg, rec->pg, rec->size - offsetof(configRecord_t, pg), rec->version)) {
                success = false;
            }
        } else {
            pgReset(reg);
            success = false;
        }
    }
    return success;
}

bool isEEPROMContentValid(void) {
    const uint8_t* p = __config_start;
    const configHeader_t* header = (const configHeader_t*)p;

#if defined(DEBUG_PG)
    print("-[Starting EEPROM validation]\r\n");
#endif

    if (header->eepromConfigVersion != EEPROM_CONF_VERSION) {
#if defined(DEBUG_PG)
        print("-[Invalid config version: expected %d, got %d]\r\n", EEPROM_CONF_VERSION,
              header->eepromConfigVersion);
#endif
        return false;
    }
    if (header->magic_be != 0xBE) {
#if defined(DEBUG_PG)
        print("-[Invalid magic number: expected 0xBE, got 0x%02X]\r\n", header->magic_be);
#endif
        return false;
    }

#if defined(DEBUG_PG)
    print("-[Header validated successfully]\r\n");
#endif

    uint16_t crc = CRC_START_VALUE;
    crc = crc16_ccitt_update(crc, header, sizeof(*header));
    p += sizeof(*header);

#if defined(DEBUG_PG)
    print("-[Header CRC]: 0x%04X\r\n", crc);
#endif

    for (;;) {
        const configRecord_t* record = (const configRecord_t*)p;

        if (record->size == 0) {
            // Found the end.  Stop scanning.
#if defined(DEBUG_PG)
            print("-[End of records reached]\r\n");
#endif
            break;
        }
        if (p + record->size >= __config_end || record->size < sizeof(*record)) {
            // Too big or too small.
#if defined(DEBUG_PG)
            print("-[Invalid record size: %d bytes]\r\n", record->size);
#endif
            return false;
        }

#if defined(DEBUG_PG)
        print("-[Processing record: pgn=%d, size=%d]\r\n", record->pgn, record->size);
#endif

        crc = crc16_ccitt_update(crc, p, record->size);

        p += record->size;
    }

    const configFooter_t* footer = (const configFooter_t*)p;
    crc = crc16_ccitt_update(crc, footer, sizeof(*footer));
    p += sizeof(*footer);

#if defined(DEBUG_PG)
    print("-[Footer CRC]: 0x%04X\r\n", crc);
#endif

    // include stored CRC in the CRC calculation
    const uint16_t* storedCrc = (const uint16_t*)p;
    crc = crc16_ccitt_update(crc, storedCrc, sizeof(*storedCrc));
    p += sizeof(storedCrc);

#if defined(DEBUG_PG)
    print("-[Stored CRC]: 0x%04X, Calculated CRC: 0x%04X]\r\n", *storedCrc, crc);
#endif

    eepromConfigSize = p - __config_start;
    // CRC has the property that if the CRC itself is included in the calculation the resulting CRC
    // will have constant value

#if defined(DEBUG_PG)
    bool valid = (crc == CRC_CHECK_VALUE);
    print("-[EEPROM validation result]: %s\r\n", valid ? "PASSED" : "FAILED");
#endif

    return crc == CRC_CHECK_VALUE;
}

bool writeSettingsToEEPROM(void) {
#if defined(DEBUG_PG)
    print("-[Starting EEPROM write]\r\n");
#endif
    config_streamer_t streamer;
    config_streamer_init(&streamer);

    config_streamer_start(&streamer, (uintptr_t)__config_start, __config_end - __config_start);

    configHeader_t header = {
        .eepromConfigVersion = EEPROM_CONF_VERSION,
        .magic_be = 0xBE,
    };

    config_streamer_write(&streamer, (uint8_t*)&header, sizeof(header));
    uint16_t crc = CRC_START_VALUE;
    crc = crc16_ccitt_update(crc, (uint8_t*)&header, sizeof(header));
#if defined(DEBUG_PG)
    print("-[Header CRC]: 0x%04X\r\n", crc);
#endif
    PG_FOREACH(reg) {
        const uint16_t regSize = pgSize(reg);
        configRecord_t record = {.size = (uint16_t)(sizeof(configRecord_t) + regSize),
                                 .pgn = pgN(reg),
                                 .version = pgVersion(reg),
                                 .flags = 0};

        record.flags |= CR_CLASSICATION_SYSTEM;
        config_streamer_write(&streamer, (uint8_t*)&record, sizeof(record));
        crc = crc16_ccitt_update(crc, (uint8_t*)&record, sizeof(record));
#if defined(DEBUG_PG)
        print("-[After record CRC]: 0x%04X\r\n", crc);
#endif
        config_streamer_write(&streamer, reg->address, regSize);
        crc = crc16_ccitt_update(crc, reg->address, regSize);
#if defined(DEBUG_PG)
        print("-[After reg data CRC]: 0x%04X\r\n", crc);
#endif
    }

    configFooter_t footer = {
        .terminator = 0,
    };

    config_streamer_write(&streamer, (uint8_t*)&footer, sizeof(footer));
    crc = crc16_ccitt_update(crc, (uint8_t*)&footer, sizeof(footer));
#if defined(DEBUG_PG)
    print("-[After footer CRC]: 0x%04X\r\n", crc);
#endif

    const uint16_t invertedBigEndianCrc = ~(((crc & 0xFF) << 8) | (crc >> 8));
#if defined(DEBUG_PG)
    print("-[Writing CRC]: 0x%04X\r\n", invertedBigEndianCrc);
#endif
    config_streamer_write(&streamer, (uint8_t*)&invertedBigEndianCrc, sizeof(crc));

    config_streamer_flush(&streamer);

    const bool success = config_streamer_finish(&streamer) == 0;
#if defined(DEBUG_PG)
    print("[Write finished with success]=%s\r\n", success ? "PASSED" : "FAILED");
#endif
    return success;
}

void writeConfigToEEPROM(void) {
    bool success = false;
    // write it
    for (int attempt = 0; attempt < 3 && !success; attempt++) {
        if (writeSettingsToEEPROM()) {
            success = true;
        }
    }

    if (success && isEEPROMContentValid()) {
        print("Configuration saved successfully!\r\n");
        return;
    }

    print("Failed to save configuration!\r\n");

    // Flash write failed - just die now
    // failureMode(FAILURE_FLASH_WRITE_FAILED);
}
