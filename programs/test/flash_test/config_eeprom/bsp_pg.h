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

#ifndef BSP_PG_H
#define BSP_PG_H

#include "bsp_eeprom.h"
#include "bsp_pg_ids.h"
#include "stdio.h"

#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2 * !!(condition)]))
#define PG_PACKED __attribute__((packed))
typedef uint16_t pgn_t;

// parameter group registry flags
typedef enum { PGRF_NONE = 0, PGRF_CLASSIFICATON_BIT = (1 << 0) } pgRegistryFlags_e;

typedef enum {
    CR_CLASSICATION_SYSTEM = 0,
    CR_CLASSICATION_PROFILE_LAST = CR_CLASSICATION_SYSTEM,
} configRecordFlags_e;

typedef enum {
    PGR_PGN_MASK = 0x0fff,
    PGR_PGN_VERSION_MASK = 0xf000,
    PGR_SIZE_MASK = 0x0fff,
    PGR_SIZE_SYSTEM_FLAG = 0x0000  // documentary
} pgRegistryInternal_e;

#define CR_CLASSIFICATION_MASK (0x3)
#define CRC_START_VALUE 0xFFFF
#define CRC_CHECK_VALUE 0x1D0F  // pre-calculated value of CRC that includes the CRC itself

typedef void(pgResetFunc)(void* /* base */, int /* size */);
typedef struct pgRegistry_s {
    pgn_t pgn;         // The parameter group number, the top 4 bits are reserved for version
    uint16_t size;     // Size of the group in RAM, the top 4 bits are reserved for flags
    uint8_t* address;  // Address of the group in RAM.
    uint8_t* copy;     // Address of the copy in RAM.
    uint8_t** ptr;     // The pointer to update after loading the record into ram.
    union {
        void* ptr;        // Pointer to init template
        pgResetFunc* fn;  // Pointer to pgResetFunc
    } reset;
} pgRegistry_t;

static inline uint16_t pgN(const pgRegistry_t* reg) {
    return reg->pgn & PGR_PGN_MASK;
}
static inline uint8_t pgVersion(const pgRegistry_t* reg) {
    return (uint8_t)(reg->pgn >> 12);
}
static inline uint16_t pgSize(const pgRegistry_t* reg) {
    return reg->size & PGR_SIZE_MASK;
}

#define PG_PACKED __attribute__((packed))

// 添加PG注册表属性定义
#ifdef __APPLE__
extern const pgRegistry_t __pg_registry_start[] __asm("section$start$__DATA$__pg_registry");
extern const pgRegistry_t __pg_registry_end[] __asm("section$end$__DATA$__pg_registry");
#define PG_REGISTER_ATTRIBUTES __attribute__((section("__DATA,__pg_registry"), used, aligned(4)))

extern const uint8_t __pg_resetdata_start[] __asm("section$start$__DATA$__pg_resetdata");
extern const uint8_t __pg_resetdata_end[] __asm("section$end$__DATA$__pg_resetdata");
#define PG_RESETDATA_ATTRIBUTES __attribute__((section("__DATA,__pg_resetdata"), used, aligned(2)))
#else
// 对于STM32等嵌入式系统，使用以下定义
extern const pgRegistry_t __pg_registry_start[];
extern const pgRegistry_t __pg_registry_end[];
#define PG_REGISTER_ATTRIBUTES __attribute__((section(".pg_registry"), used, aligned(4)))

extern const uint8_t __pg_resetdata_start[];
extern const uint8_t __pg_resetdata_end[];
#define PG_RESETDATA_ATTRIBUTES __attribute__((section(".pg_resetdata"), used, aligned(2)))

#endif

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

#ifndef __UNIQL
#define __UNIQL_CONCAT2(x, y) x##y
#define __UNIQL_CONCAT(x, y) __UNIQL_CONCAT2(x, y)
#define __UNIQL(x) __UNIQL_CONCAT(x, __LINE__)
#endif

// overwrite _name with data passed as arguments. This version forces GCC to really copy data
// It is not possible to use multiple RESET_CONFIGs on single line (__UNIQL limitation)
#define RESET_CONFIG(_type, _name, ...)                            \
    static const _type __UNIQL(_reset_template_) = {__VA_ARGS__};  \
    memcpy((_name), &__UNIQL(_reset_template_), sizeof(*(_name))); \
    /**/

// overwrite _name with data passed as arguments. GCC is allowed to set structure field-by-field
#define RESET_CONFIG_2(_type, _name, ...) \
    *(_name) = (_type){__VA_ARGS__};      \
    /**/

// Helper to iterate over the PG register.  Cheaper than a visitor style callback.
#define PG_FOREACH(_name) \
    for (const pgRegistry_t* _name = __pg_registry_start; _name < __pg_registry_end; _name++)

// Reset configuration to default (by name)
#define PG_RESET(_name)                             \
    do {                                            \
        extern const pgRegistry_t _name##_Registry; \
        pgReset(&_name##_Registry);                 \
    } while (0) /**/

// Declare system config
#define PG_DECLARE(_type, _name)                \
    extern _type _name##_System;                \
    extern _type _name##_Copy;                  \
    static inline const _type* _name(void) {    \
        return &_name##_System;                 \
    }                                           \
    static inline _type* _name##Mutable(void) { \
        return &_name##_System;                 \
    }                                           \
    struct _dummy /**/

// Declare system config array
#define PG_DECLARE_ARRAY(_type, _size, _name)          \
    extern _type _name##_SystemArray[_size];           \
    extern _type _name##_CopyArray[_size];             \
    static inline const _type* _name(int _index) {     \
        return &_name##_SystemArray[_index];           \
    }                                                  \
    static inline _type* _name##Mutable(int _index) {  \
        return &_name##_SystemArray[_index];           \
    }                                                  \
    static inline _type(*_name##_array(void))[_size] { \
        return &_name##_SystemArray;                   \
    }                                                  \
    struct _dummy /**/

// Register system config
#define PG_REGISTER_I(_type, _name, _pgn, _version, _reset)        \
    _type _name##_System;                                          \
    _type _name##_Copy;                                            \
    /* Force external linkage for g++. Catch multi registration */ \
    extern const pgRegistry_t _name##_Registry;                    \
    const pgRegistry_t _name##_Registry PG_REGISTER_ATTRIBUTES = { \
        .pgn = _pgn | (_version << 12),                            \
        .size = sizeof(_type) | PGR_SIZE_SYSTEM_FLAG,              \
        .address = (uint8_t*)&_name##_System,                      \
        .copy = (uint8_t*)&_name##_Copy,                           \
        .ptr = 0,                                                  \
        _reset,                                                    \
    } /**/

#define PG_REGISTER(_type, _name, _pgn, _version)                    \
    PG_REGISTER_I(_type, _name, _pgn, _version, .reset = {.ptr = 0}) \
    /**/

#define PG_REGISTER_WITH_RESET_FN(_type, _name, _pgn, _version)                                    \
    extern void pgResetFn_##_name(_type*);                                                         \
    PG_REGISTER_I(_type, _name, _pgn, _version, .reset = {.fn = (pgResetFunc*)&pgResetFn_##_name}) \
    /**/

#define PG_REGISTER_WITH_RESET_TEMPLATE(_type, _name, _pgn, _version)                              \
    extern const _type pgResetTemplate_##_name;                                                    \
    PG_REGISTER_I(_type, _name, _pgn, _version, .reset = {.ptr = (void*)&pgResetTemplate_##_name}) \
    /**/

// Register system config array
#define PG_REGISTER_ARRAY_I(_type, _size, _name, _pgn, _version, _reset) \
    _type _name##_SystemArray[_size];                                    \
    _type _name##_CopyArray[_size];                                      \
    extern const pgRegistry_t _name##_Registry;                          \
    const pgRegistry_t _name##_Registry PG_REGISTER_ATTRIBUTES = {       \
        .pgn = _pgn | (_version << 12),                                  \
        .size = (sizeof(_type) * _size) | PGR_SIZE_SYSTEM_FLAG,          \
        .address = (uint8_t*)&_name##_SystemArray,                       \
        .copy = (uint8_t*)&_name##_CopyArray,                            \
        .ptr = 0,                                                        \
        _reset,                                                          \
    } /**/

#define PG_REGISTER_ARRAY(_type, _size, _name, _pgn, _version)                    \
    PG_REGISTER_ARRAY_I(_type, _size, _name, _pgn, _version, .reset = {.ptr = 0}) \
    /**/

#define PG_REGISTER_ARRAY_WITH_RESET_FN(_type, _size, _name, _pgn, _version) \
    extern void pgResetFn_##_name(_type*);                                   \
    PG_REGISTER_ARRAY_I(_type, _size, _name, _pgn, _version,                 \
                        .reset = {.fn = (pgResetFunc*)&pgResetFn_##_name})   \
    /**/

#if 0
// ARRAY reset mechanism is not implemented yet, only few places in code would benefit from it - See pgResetInstance
#define PG_REGISTER_ARRAY_WITH_RESET_TEMPLATE(_type, _size, _name, _pgn, _version) \
    extern const _type pgResetTemplate_##_name;                                    \
    PG_REGISTER_ARRAY_I(_type, _size, _name, _pgn, _version,                       \
                        .reset = {.ptr = (void*)&pgResetTemplate_##_name})         \
    /**/
#endif

// Emit reset defaults for config.
// Config must be registered with PG_REGISTER_<xxx>_WITH_RESET_TEMPLATE macro
#define PG_RESET_TEMPLATE(_type, _name, ...) \
    const _type pgResetTemplate_##_name PG_RESETDATA_ATTRIBUTES = {__VA_ARGS__} /**/

#define CONVERT_PARAMETER_TO_FLOAT(param) (0.001f * param)
#define CONVERT_PARAMETER_TO_PERCENT(param) (0.01f * param)

const pgRegistry_t* pgFind(pgn_t pgn);
void pgLoad(const pgRegistry_t* reg, const void* from, int size, int version);
int pgStore(const pgRegistry_t* reg, void* to, int size);
void pgResetAll(void);
void pgResetInstance(const pgRegistry_t* reg, uint8_t* base);
bool pgResetCopy(uint8_t* copy, pgn_t pgn);
void pgReset(const pgRegistry_t* reg);

#define EEPROM_CONF_VERSION 168

bool isEEPROMContentValid(void);
bool loadEEPROM(void);
void writeConfigToEEPROM(void);
uint16_t getEEPROMConfigSize(void);

#endif  // BSP_PG_H
