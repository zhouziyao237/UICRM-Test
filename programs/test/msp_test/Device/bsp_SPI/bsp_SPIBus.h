/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#pragma once

#include "spi.h"
#include "bsp_spi.h"

// 当前版本功能说明:
// 1. 仅支持SPI主机模式
// 2. 支持BLOCKED、INTERRUPT和DMA三种传输模式
// 3. 支持单设备和多设备管理
// 4. 支持自动CS控制
// 5. 暂不支持SPI从机模式(需要底层bsp::SPI扩展支持)

/* spi callback function pointer */
typedef void (*spi_rx_callback_t)(void* args);

typedef struct spiPort_s spiPort_t;

// todo: SPI模式定义
typedef enum {
    SPI_MODE_BLOCKED = 0,
    SPI_MODE_INTURRUPT = 1,
    SPI_MODE_DMA = 2,
} spi_mode_e;

// todo: SPI主设备状态
typedef enum {
    SPI_MASTER_STATUS_OK = 0,
    SPI_MASTER_STATUS_BUSY = 1,
    SPI_MASTER_STATUS_ERROR = 2
} spi_master_status_e;

// todo: 虚函数表，定义了SPI的各种操作
struct spiPortVTable {
    void (*transmitReceive)(spiPort_t* instance, uint8_t* tx_data, uint8_t* rx_data, uint32_t length);
    void (*transmit)(spiPort_t* instance, uint8_t* tx_data, uint32_t length);
    void (*receive)(spiPort_t* instance, uint8_t* rx_data, uint32_t length);
    bool (*isBusy)(spiPort_t* instance);
    void (*abort)(spiPort_t* instance);
    void (*setMode)(spiPort_t* instance, spi_mode_e mode);
    void (*registerCallback)(spiPort_t* instance, spi_rx_callback_t callback, void* args);
    bool (*isDMA)(spiPort_t* instance);
};

// todo: SPI结构体
typedef struct spiPort_s {
    const struct spiPortVTable* vTable;
    void* privateData;
} spiPort_t;

// todo: SPI设备结构体
typedef struct spiDevice_s {
    void* privateData;
} spiDevice_t;

// todo: SPI主设备结构体
typedef struct spiMaster_s {
    void* privateData;
} spiMaster_t;

spiPort_t* newSPIPort(SPI_HandleTypeDef* hspi, spi_mode_e mode);
spiDevice_t* newSPIDevice(spiPort_t* spi, void* cs);
spiMaster_t* newSPIMaster(spiPort_t* spi);

// todo: 公共接口函数
void spiTransmitReceive(spiPort_t* instance, uint8_t* tx_data, uint8_t* rx_data, uint32_t length);
void spiTransmit(spiPort_t* instance, uint8_t* tx_data, uint32_t length);
void spiReceive(spiPort_t* instance, uint8_t* rx_data, uint32_t length);
bool spiIsBusy(spiPort_t* instance);
void spiAbort(spiPort_t* instance);
void spiSetMode(spiPort_t* instance, spi_mode_e mode);
void spiRegisterCallback(spiPort_t* instance, spi_rx_callback_t callback, void* args);
bool spiIsDMA(spiPort_t* instance);

// todo: SPI设备接口函数
void spiDeviceRegisterCallback(spiDevice_t* device, void (*callback)(void* args), void* args);
void spiDevicePrepareTransmit(spiDevice_t* device);
void spiDeviceFinishTransmit(spiDevice_t* device);
bool spiDeviceIsTransmitting(spiDevice_t* device);

// todo: SPI主设备接口函数
spiDevice_t* newSPIDeviceViaMaster(spiMaster_t* master, void* cs);
spiDevice_t* spiMasterNewDevice(spiMaster_t* master, void* cs);
void spiMasterAddDevice(spiMaster_t* master, spiDevice_t* device);
bool spiMasterIsBusy(spiMaster_t* master);
spi_master_status_e spiMasterTransmit(spiMaster_t* master, spiDevice_t* device, uint8_t* tx_data, uint32_t length);
spi_master_status_e spiMasterReceive(spiMaster_t* master, spiDevice_t* device, uint8_t* rx_data, uint32_t length);
spi_master_status_e spiMasterTransmitReceive(spiMaster_t* master, spiDevice_t* device, uint8_t* tx_data, uint8_t* rx_data, uint32_t length);
void spiMasterSetMode(spiMaster_t* master, spi_mode_e mode);
void spiMasterSetAutoCS(spiMaster_t* master, bool auto_cs);

// todo: 释放函数
void deleteSPIPort(spiPort_t* instance);
void deleteSPIDevice(spiDevice_t* device);
void deleteSPIMaster(spiMaster_t* master);
