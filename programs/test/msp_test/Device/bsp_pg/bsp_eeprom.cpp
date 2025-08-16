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

#include "bsp_eeprom.h"

#include <string.h>

#include "bsp_print.h"
#include "stdio.h"
#include "target.h"
#include "usart.h"

#if defined(STM32F4)
/*
Sector 0    0x08000000 - 0x08003FFF 16 Kbytes
Sector 1    0x08004000 - 0x08007FFF 16 Kbytes
Sector 2    0x08008000 - 0x0800BFFF 16 Kbytes
Sector 3    0x0800C000 - 0x0800FFFF 16 Kbytes
Sector 4    0x08010000 - 0x0801FFFF 64 Kbytes
Sector 5    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 6    0x08040000 - 0x0805FFFF 128 Kbytes
Sector 7    0x08060000 - 0x0807FFFF 128 Kbytes
Sector 8    0x08080000 - 0x0809FFFF 128 Kbytes
Sector 9    0x080A0000 - 0x080BFFFF 128 Kbytes
Sector 10   0x080C0000 - 0x080DFFFF 128 Kbytes
Sector 11   0x080E0000 - 0x080FFFFF 128 Kbytes
*/

uint32_t getFLASHSectorForEEPROM(void) {
    if ((uint32_t)&__config_start <= 0x08003FFF)
        return FLASH_SECTOR_0;
    if ((uint32_t)&__config_start <= 0x08007FFF)
        return FLASH_SECTOR_1;
    if ((uint32_t)&__config_start <= 0x0800BFFF)
        return FLASH_SECTOR_2;
    if ((uint32_t)&__config_start <= 0x0800FFFF)
        return FLASH_SECTOR_3;
    if ((uint32_t)&__config_start <= 0x0801FFFF)
        return FLASH_SECTOR_4;
    if ((uint32_t)&__config_start <= 0x0803FFFF)
        return FLASH_SECTOR_5;
    if ((uint32_t)&__config_start <= 0x0805FFFF)
        return FLASH_SECTOR_6;
    if ((uint32_t)&__config_start <= 0x0807FFFF)
        return FLASH_SECTOR_7;
    if ((uint32_t)&__config_start <= 0x0809FFFF)
        return FLASH_SECTOR_8;
    if ((uint32_t)&__config_start <= 0x080DFFFF)
        return FLASH_SECTOR_9;
    if ((uint32_t)&__config_start <= 0x080BFFFF)
        return FLASH_SECTOR_10;
    if ((uint32_t)&__config_start <= 0x080FFFFF)
        return FLASH_SECTOR_11;
    // Not good
    return HAL_ERROR;
}

#endif

/*-------------------------------------------------*/
/*函数名：内部eeprom擦除功能                          */
/*参  数：Address：擦除地址                        */
/*参  数：wData：擦除数据缓冲区                     */
/*参  数：len：擦除数据总长                         */
/*返回值：无                                       */
/*-------------------------------------------------*/

HAL_StatusTypeDef EEPROM_EraseSector(uint32_t Sector) {
    HAL_StatusTypeDef status = HAL_OK;

    // 检查扇区是否有效
    if (Sector == HAL_ERROR) {
        return HAL_ERROR;
    }

    // 对于STM32F4，Flash操作需要先解锁
    HAL_FLASH_Unlock();

    // 擦除需要的扇区
    uint32_t sector_error = 0;
    FLASH_EraseInitTypeDef erase_init;

    // 根据地址确定需要擦除的扇区
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase_init.NbSectors = 1;
    erase_init.Sector = Sector;

    HAL_FLASH_Unlock();
    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    HAL_FLASH_Lock();
    if (status != HAL_OK) {
#if defined(DEBUG_FLASH)
        print("Erase Fail!!!\r\n");  // 串口提示写入错误
#endif
        return HAL_ERROR;
    }
#if defined(DEBUG_FLASH)
    print("Erase Success.\r\n\r\n");
#endif
    return HAL_OK;
}

// HAL_StatusTypeDef EEPROM_ProgramWord(uint32_t Address, uint32_t Data) {
//     HAL_StatusTypeDef status = HAL_OK;
//     // 检查地址范围
//     if ((Address < DATA_EEPROM_START_ADDR) || (Address + 4 > DATA_EEPROM_END_ADDR)) {
//         print("Error: Address out of range.\r\n");
//         return HAL_ERROR;
//     }
//     HAL_FLASH_Unlock();  // 解锁Flash
//     // 使用HAL_FLASH_Program函数进行编程
//     status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Data);
//     if (status != HAL_OK) {
//         print("Error: Program Fail!!!\r\n");
//         return HAL_ERROR;
//     }
//     HAL_FLASH_Lock();  // 上锁
//
// #if defined(DEBUG_FLASH)
//     print("Program Success.\r\n");
// #endif
//     return status;
// }

// ... existing code ...
HAL_StatusTypeDef EEPROM_ProgramWord(uint32_t Address, uint32_t Data) {
    HAL_StatusTypeDef status = HAL_OK;

    // 检查地址范围
    if ((Address < DATA_EEPROM_START_ADDR) || (Address + 4 > DATA_EEPROM_END_ADDR)) {
        return HAL_ERROR;
    }

    // 解锁Flash
    HAL_FLASH_Unlock();

    // 清除所有错误标志
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    // 使用HAL_FLASH_Program函数进行编程
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Data);

    // 等待操作完成
    if (status == HAL_OK) {
        // 等待Flash操作完成
        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET) {
            // 等待
        }
    }

    // 检查是否有错误
    if (__HAL_FLASH_GET_FLAG(FLASH_SR_RDERR) != RESET) {
        // 清除错误标志
        __HAL_FLASH_CLEAR_FLAG(FLASH_SR_RDERR);
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    // 上锁Flash
    HAL_FLASH_Lock();

    return status;
}
// ... existing code ...

static int write_word(config_streamer_t* c, uint32_t value) {
    static HAL_StatusTypeDef status = HAL_OK;

    if (c->err != 0) {
        return c->err;
    }

    if (c->address % FLASH_PAGE_SIZE == 0) {
        const uint32_t Sector = getFLASHSectorForEEPROM();
        if (Sector == HAL_ERROR) {
            print("Error: getFLASHSectorForEEPROM Fail!!!\r\n");
            return -3;
        }

        status = EEPROM_EraseSector(Sector);
        if (status != HAL_OK) {
            return -1;
        }
    }

    status = EEPROM_ProgramWord(c->address, value);
    if (status != HAL_OK) {
        return -2;
    }

    c->address += sizeof(value);
    return 0;
}

void config_streamer_init(config_streamer_t* c) {
    memset(c, 0, sizeof(*c));
}

int config_streamer_write(config_streamer_t* c, const uint8_t* p, uint32_t size) {
    for (const uint8_t* pat = p; pat != (uint8_t*)p + size; pat++) {
        c->buffer.b[c->at++] = *pat;

        if (c->at == sizeof(c->buffer)) {
            c->err = write_word(c, c->buffer.w);
            c->at = 0;
        }
    }
    return c->err;
}

void config_streamer_start(config_streamer_t* c, uintptr_t base, int size) {
    // base must start at FLASH_PAGE_SIZE boundary
    c->address = base;
    c->size = size;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    c->err = 0;
}

int config_streamer_status(config_streamer_t* c) {
    return c->err;
}

int config_streamer_flush(config_streamer_t* c) {
    if (c->at != 0) {
        memset(c->buffer.b + c->at, 0, sizeof(c->buffer) - c->at);
        c->err = write_word(c, c->buffer.w);
        c->at = 0;
    }
    return c->err;
}

int config_streamer_finish(config_streamer_t* c) {
    return c->err;
}