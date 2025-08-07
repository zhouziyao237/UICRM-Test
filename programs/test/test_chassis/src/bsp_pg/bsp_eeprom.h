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

#ifndef BSP_EEPROM_H
#define BSP_EEPROM_H

#include "main.h"
#include "stm32f4xx.h"  //包含需要的头文件
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

// 配置数据区域符号
extern const uint8_t __config_start[];
extern const uint8_t __config_end[];
#define PG_REGISTRY_SIZE (__pg_registry_end - __pg_registry_start)

// 定义STM32F4的页大小
#define STM32F427_437xx
#define STM32F4

// F4
# if defined(STM32F40_41xxx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000) // 16K sectors
# elif defined (STM32F411xE)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000)
# elif defined(STM32F427_437xx)
#  define FLASH_PAGE_SIZE                 ((uint32_t)0x4000)
# elif defined (STM32F446xx)
#  define FLASH_PAGE_SIZE
#endif

#define PROGRAM_START_ADDR 0x08000000      // 程序起始地址
#define DATA_EEPROM_START_ADDR 0x080E0000  // 配置数据起始地址
#define DATA_EEPROM_END_ADDR (DATA_EEPROM_START_ADDR + FLASH_PAGE_SIZE)  // 配置数据结束地址
#define iEEPROM_CHECK_NUM 2

#define GET_SECTOR_FOR_EEPROM(Sector)             \
    (((Sector) == FLASH_SECTOR_0)    ? 0x08000000 \
     : ((Sector) == FLASH_SECTOR_1)  ? 0x08004000 \
     : ((Sector) == FLASH_SECTOR_2)  ? 0x08008000 \
     : ((Sector) == FLASH_SECTOR_3)  ? 0x0800C000 \
     : ((Sector) == FLASH_SECTOR_4)  ? 0x08010000 \
     : ((Sector) == FLASH_SECTOR_5)  ? 0x08020000 \
     : ((Sector) == FLASH_SECTOR_6)  ? 0x08040000 \
     : ((Sector) == FLASH_SECTOR_7)  ? 0x08060000 \
     : ((Sector) == FLASH_SECTOR_8)  ? 0x08080000 \
     : ((Sector) == FLASH_SECTOR_9)  ? 0x080A0000 \
     : ((Sector) == FLASH_SECTOR_10) ? 0x080C0000 \
     : ((Sector) == FLASH_SECTOR_11) ? 0x080E0000 \
                                     : 0xFFFFFFFF)

#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))
#define ARRAYEND(x) (&(x)[ARRAYLEN(x)])

typedef struct config_streamer_s {
    uintptr_t address;
    int size;
    union {
        uint8_t b[4];
        uint32_t w;
    } buffer;
    int at;
    int err;
    bool unlocked;
} config_streamer_t;

HAL_StatusTypeDef EEPROM_EraseSector(uint32_t Sector);
HAL_StatusTypeDef EEPROM__ProgramWord(uint32_t Address, uint32_t Data);

void config_streamer_init(config_streamer_t* c);

void config_streamer_start(config_streamer_t* c, uintptr_t base, int size);
int config_streamer_write(config_streamer_t* c, const uint8_t* p, uint32_t size);
int config_streamer_flush(config_streamer_t* c);

int config_streamer_finish(config_streamer_t* c);
int config_streamer_status(config_streamer_t* c);

#endif  // BSP_EEPROM_H
