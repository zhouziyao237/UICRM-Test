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

static uint32_t getFLASHSectorForEEPROM(void) {
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

HAL_StatusTypeDef EEPROM_EraseData(uint32_t start, uint32_t size) {
    uint32_t Address = start;
    HAL_StatusTypeDef status = HAL_OK;

    // print("EEPROM_EraseData len:%d\r\n", size);

    // STM32F4没有专门的EEPROM，使用Flash模拟EEPROM
    // 这里我们不实际擦除，而是在写入时处理
    // 只需确保地址在有效范围内
    if ((Address < DATA_EEPROM_START_ADDR) || (Address + size * 4 > DATA_EEPROM_END_ADDR)) {
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
    erase_init.Sector = getFLASHSectorForEEPROM();

    // 检查扇区是否有效
    if (getFLASHSectorForEEPROM() == HAL_ERROR) {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

    HAL_FLASH_Lock();

    if (status != HAL_OK) {
        // print("Erase Fail!!!\r\n");  // 串口提示写入错误
        return HAL_ERROR;
    }

    // print("Erase Success.\r\n\r\n");
    return HAL_OK;
}

/*-------------------------------------------------*/
/*函数名：内部eeprom写功能                          */
/*参  数：Address：写入地址                        */
/*参  数：wData：写入数据缓冲区                     */
/*参  数：len：写入数据总长                         */
/*返回值：无                                       */
/*-------------------------------------------------*/
HAL_StatusTypeDef EEPROM_WriteData(uint32_t Address, uint32_t* wData, uint32_t len) {
    uint32_t i;
    HAL_StatusTypeDef status = HAL_OK;

    // 检查地址范围
    if ((Address < DATA_EEPROM_START_ADDR) || (Address + len * 4 > DATA_EEPROM_END_ADDR)) {
        return HAL_ERROR;
    }

    HAL_FLASH_Unlock();  // 解锁Flash

    // 先擦除扇区
    status = EEPROM_EraseData(Address, len);
    if (status != HAL_OK) {
        return HAL_ERROR;
    }
    // print("Write Address:%d\r\n", Address);
    // print("Write Data:\r\n");

    for (i = 0; i < len; i++)  // for循环，需要写入多少数据，就循环几次
    {
        // print("wData[%d]=%08x\r\n", i, wData[i]);

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address + i * 4, *(wData + i));
        if (status != HAL_OK)  // 调用写数据函数，如果返回的不是HAL_OK，表示写入出错，进入if
        {
            // print("Write Error!!!\r\n");  // 串口提示写入错误
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
        Address += 4;  // 地址递增4，因为一次写一个字，是4个字节
    }
    // print("Write Success.\r\n\r\n");

    HAL_FLASH_Lock();  // 上锁
    return HAL_OK;
}

/*-------------------------------------------------*/
/*函数名：内部eeprom读功能                        */
/*参  数：Address：读取地址                        */
/*参  数：rData：保存数据缓冲区                     */
/*参  数：len：读取数据总长                         */
/*返回值：无                                       */
/*-------------------------------------------------*/
HAL_StatusTypeDef EEPROM_ReadData(uint32_t Address, uint32_t* rData, uint32_t len) {
    uint32_t i;
    uint32_t* wAddr = 0;

    // 检查地址范围
    if ((Address < DATA_EEPROM_START_ADDR) || (Address + len * 4 > DATA_EEPROM_END_ADDR)) {
        return HAL_ERROR;
    }

    wAddr = (uint32_t*)(Address);  // 转换地址，地址从0x08080000开始
    // print("Read Address:%d\r\n", Address);

    // print("Read Data:\r\n");
    for (i = 0; i < len; i++) {  // for循环，需要读取多少数据，就循环几次
        *rData++ = *wAddr++;     // 每次读取的数据保存到rData缓冲区
        // print("rData[%d]=%08x\r\n",i,rData[i]);
    }
    // print("Read Complete.\r\n");
    return HAL_OK;
}

/*-------------------------------------------------*/
/*函数名：带有校验操作的内部eeprom写功能                          */
/*参  数：Address：写入地址                        */
/*参  数：wData：写入数据缓冲区                     */
/*参  数：len：写入数据总长                         */
/*返回值：无                                       */
/*-------------------------------------------------*/
HAL_StatusTypeDef EEPROM_WRITE_Verify_CHECK(uint32_t Address, uint32_t* wData, uint32_t len) {
    uint32_t buff[len];
    uint32_t i;
    for (i = 0; i < iEEPROM_CHECK_NUM; i++) {
        if (EEPROM_WriteData(Address, wData, len) == HAL_OK) {
            if (EEPROM_ReadData(Address, buff, len) == HAL_OK) {
                if (memcmp(wData, buff, len * sizeof(uint32_t)) == 0) {
                    // print("\r\nWRITE_Verify Completing Comparative\r\n\r\n");
                    return HAL_OK;
                }
            }
        }
    }
    return HAL_ERROR;
}

/*-------------------------------------------------*/
/*函数名：带有校验操作的内部eeprom读功能                        */
/*参  数：Address：读取地址                        */
/*参  数：rData：保存数据缓冲区                     */
/*参  数：len：读取数据总长                         */
/*返回值：无                                       */
/*-------------------------------------------------*/
HAL_StatusTypeDef EEPROM_Read_Verify_CHECK(uint32_t Address, uint32_t* rData, uint32_t len) {
    uint32_t buff0[len];
    uint32_t buff1[len];
    uint8_t i, j;

    //	print("len:%d\r\n", len);
    //	print("Address:%d\r\n", Address);

    for (i = 0; i < iEEPROM_CHECK_NUM; i++) {
        // print("First  read Verify\r\n");
        if (EEPROM_ReadData(Address, buff0, len) != HAL_OK) {
            continue;
        }
        // print("Second read Verify\r\n");
        if (EEPROM_ReadData(Address, buff1, len) != HAL_OK) {
            continue;
        }

        if (memcmp(buff0, buff1, len * sizeof(uint32_t)) == 0) {
            // print("Read_Verify Completing Comparative\r\n");
            for (j = 0; j < len; j++) {
                *rData++ = buff0[j];
            }
            return HAL_OK;
        }
    }
    return HAL_ERROR;
}

/*-------------------------------------------------*/

// FLASH_Status FLASH_EraseSector(uint32_t Sector) {
//     // 使用已有的EEPROM_EraseData函数实现扇区擦除功能
//     // 首先根据扇区号计算起始地址
//
//     uint32_t address = GET_SECTOR_FOR_EEPROM(Sector);
//
//     // 调用已有的EEPROM_EraseData函数，擦除1个扇区
//     HAL_StatusTypeDef status = EEPROM_EraseData(address, 1);
//
//     // 转换返回值以匹配FLASH_Status
//     switch (status) {
//         case HAL_OK:
//             return COMPLETE;
//         case HAL_ERROR:
//             return ERROR_PG;
//         default:
//             return ERROR_WRP;
//     }
// }

static int write_word(config_streamer_t* c, uint32_t value) {
    if (c->err != 0) {
        return c->err;
    }

    if (c->address % FLASH_PAGE_SIZE == 0) {
        // 使用已实现的FLASH_EraseSector函数擦除扇区
        const FLASH_Status status = FLASH_EraseSector(getFLASHSectorForEEPROM());

        if (status != COMPLETE) {
            return -1;
        }
    }

    // 使用已有的EEPROM_WriteData函数进行编程
    HAL_StatusTypeDef status = EEPROM_WriteData(c->address, &value, sizeof(value)/sizeof(uint32_t));

    // 检查编程是否成功
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

    // FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
    // FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

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