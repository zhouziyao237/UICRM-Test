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

#include "bsp_SPIBus.h"

#include <stdlib.h>
#include <string.h>

#include "bsp_spi.h"

// 私有数据结构
typedef struct {
    bsp::SPI* spi_instance;
} spiPrivateData_t;

typedef struct {
    bsp::SPIDevice* spi_device_instance;
} spiDevicePrivateData_t;

typedef struct {
    bsp::SPIMaster* spi_master_instance;
    bsp::SPI* spi_instance;  // 保存SPI实例的引用
} spiMasterPrivateData_t;

// VTable函数实现
static void spiTransmitReceiveImpl(spiPort_t* instance, uint8_t* tx_data, uint8_t* rx_data,
                                   uint32_t length) {
    spiPrivateData_t* privateData = (spiPrivateData_t*)instance->privateData;
    if (privateData && privateData->spi_instance) {
        privateData->spi_instance->TransmitReceive(tx_data, rx_data, length);
    }
}

static void spiTransmitImpl(spiPort_t* instance, uint8_t* tx_data, uint32_t length) {
    spiPrivateData_t* privateData = (spiPrivateData_t*)instance->privateData;
    if (privateData && privateData->spi_instance) {
        privateData->spi_instance->Transmit(tx_data, length);
    }
}

static void spiReceiveImpl(spiPort_t* instance, uint8_t* rx_data, uint32_t length) {
    spiPrivateData_t* privateData = (spiPrivateData_t*)instance->privateData;
    if (privateData && privateData->spi_instance) {
        privateData->spi_instance->Receive(rx_data, length);
    }
}

static bool spiIsBusyImpl(spiPort_t* instance) {
    spiPrivateData_t* privateData = (spiPrivateData_t*)instance->privateData;
    if (privateData && privateData->spi_instance) {
        return privateData->spi_instance->IsBusy();
    }
    return false;
}

static void spiAbortImpl(spiPort_t* instance) {
    spiPrivateData_t* privateData = (spiPrivateData_t*)instance->privateData;
    if (privateData && privateData->spi_instance) {
        privateData->spi_instance->Abort();
    }
}

static void spiSetModeImpl(spiPort_t* instance, spi_mode_e mode) {
    spiPrivateData_t* privateData = (spiPrivateData_t*)instance->privateData;
    if (privateData && privateData->spi_instance) {
        privateData->spi_instance->SetMode((bsp::spi_mode_e)mode);
    }
}

static void spiRegisterCallbackImpl(spiPort_t* instance, spi_rx_callback_t callback, void* args) {
    spiPrivateData_t* privateData = (spiPrivateData_t*)instance->privateData;
    if (privateData && privateData->spi_instance) {
        privateData->spi_instance->RegisterCallback((bsp::spi_rx_callback_t)callback, args);
    }
}

static bool spiIsDMAImpl(spiPort_t* instance) {
    spiPrivateData_t* privateData = (spiPrivateData_t*)instance->privateData;
    if (privateData && privateData->spi_instance) {
        return privateData->spi_instance->IsDMA();
    }
    return false;
}

// SPI VTable
static const struct spiPortVTable spiVTable = {
    spiTransmitReceiveImpl, spiTransmitImpl, spiReceiveImpl,          spiIsBusyImpl,
    spiAbortImpl,           spiSetModeImpl,  spiRegisterCallbackImpl, spiIsDMAImpl};

// 公共接口函数实现
void spiTransmitReceive(spiPort_t* instance, uint8_t* tx_data, uint8_t* rx_data, uint32_t length) {
    if (!instance || !instance->vTable || !instance->vTable->transmitReceive)
        return;
    instance->vTable->transmitReceive(instance, tx_data, rx_data, length);
}

void spiTransmit(spiPort_t* instance, uint8_t* tx_data, uint32_t length) {
    if (!instance || !instance->vTable || !instance->vTable->transmit)
        return;
    instance->vTable->transmit(instance, tx_data, length);
}

void spiReceive(spiPort_t* instance, uint8_t* rx_data, uint32_t length) {
    if (!instance || !instance->vTable || !instance->vTable->receive)
        return;
    instance->vTable->receive(instance, rx_data, length);
}

bool spiIsBusy(spiPort_t* instance) {
    if (!instance || !instance->vTable || !instance->vTable->isBusy)
        return false;
    return instance->vTable->isBusy(instance);
}

void spiAbort(spiPort_t* instance) {
    if (!instance || !instance->vTable || !instance->vTable->abort)
        return;
    instance->vTable->abort(instance);
}

void spiSetMode(spiPort_t* instance, spi_mode_e mode) {
    if (!instance || !instance->vTable || !instance->vTable->setMode)
        return;
    instance->vTable->setMode(instance, mode);
}

void spiRegisterCallback(spiPort_t* instance, spi_rx_callback_t callback, void* args) {
    if (!instance || !instance->vTable || !instance->vTable->registerCallback)
        return;
    instance->vTable->registerCallback(instance, callback, args);
}

bool spiIsDMA(spiPort_t* instance) {
    if (!instance || !instance->vTable || !instance->vTable->isDMA)
        return false;
    return instance->vTable->isDMA(instance);
}

// SPI设备接口函数实现
void spiDeviceRegisterCallback(spiDevice_t* device, void (*callback)(void* args), void* args) {
    spiDevicePrivateData_t* privateData = (spiDevicePrivateData_t*)device->privateData;
    if (privateData && privateData->spi_device_instance) {
        privateData->spi_device_instance->RegisterCallback((bsp::spi_device_rx_callback_t)callback,
                                                           args);
    }
}

void spiDevicePrepareTransmit(spiDevice_t* device) {
    spiDevicePrivateData_t* privateData = (spiDevicePrivateData_t*)device->privateData;
    if (privateData && privateData->spi_device_instance) {
        privateData->spi_device_instance->PrepareTransmit();
    }
}

void spiDeviceFinishTransmit(spiDevice_t* device) {
    spiDevicePrivateData_t* privateData = (spiDevicePrivateData_t*)device->privateData;
    if (privateData && privateData->spi_device_instance) {
        privateData->spi_device_instance->FinishTransmit();
    }
}

bool spiDeviceIsTransmitting(spiDevice_t* device) {
    spiDevicePrivateData_t* privateData = (spiDevicePrivateData_t*)device->privateData;
    if (privateData && privateData->spi_device_instance) {
        return privateData->spi_device_instance->IsTransmitting();
    }
    return false;
}

// SPI主设备接口函数实现
spiDevice_t* spiMasterNewDevice(spiMaster_t* master, void* cs) {
    spiMasterPrivateData_t* privateData = (spiMasterPrivateData_t*)master->privateData;
    if (privateData && privateData->spi_master_instance) {
        // 创建新的设备结构
        spiDevice_t* device = (spiDevice_t*)malloc(sizeof(spiDevice_t));
        if (!device)
            return nullptr;

        memset(device, 0, sizeof(spiDevice_t));

        spiDevicePrivateData_t* devicePrivateData =
            (spiDevicePrivateData_t*)malloc(sizeof(spiDevicePrivateData_t));
        if (!devicePrivateData) {
            free(device);
            return nullptr;
        }

        // 使用保存的SPI实例而不是调用不存在的GetSPI方法
        bsp::SPI* spiInstance = privateData->spi_instance;
        if (!spiInstance) {
            free(devicePrivateData);
            free(device);
            return nullptr;
        }

        // 创建bsp::SPIDevice实例
        bsp::spi_device_init_t init;
        init.spi = spiInstance;
        init.cs = (bsp::GPIO*)cs;
        devicePrivateData->spi_device_instance = new bsp::SPIDevice(init);
        if (!devicePrivateData->spi_device_instance) {
            free(devicePrivateData);
            free(device);
            return nullptr;
        }

        device->privateData = devicePrivateData;
        return device;
    }
    return nullptr;
}

void spiMasterAddDevice(spiMaster_t* master, spiDevice_t* device) {
    spiMasterPrivateData_t* privateData = (spiMasterPrivateData_t*)master->privateData;
    if (privateData && privateData->spi_master_instance) {
        spiDevicePrivateData_t* devicePrivateData = (spiDevicePrivateData_t*)device->privateData;
        if (devicePrivateData && devicePrivateData->spi_device_instance) {
            privateData->spi_master_instance->AddDevice(devicePrivateData->spi_device_instance);
        }
    }
}

bool spiMasterIsBusy(spiMaster_t* master) {
    spiMasterPrivateData_t* privateData = (spiMasterPrivateData_t*)master->privateData;
    if (privateData && privateData->spi_master_instance) {
        return privateData->spi_master_instance->IsBusy();
    }
    return false;
}

spi_master_status_e spiMasterTransmit(spiMaster_t* master, spiDevice_t* device, uint8_t* tx_data,
                                      uint32_t length) {
    spiMasterPrivateData_t* privateData = (spiMasterPrivateData_t*)master->privateData;
    if (privateData && privateData->spi_master_instance) {
        spiDevicePrivateData_t* devicePrivateData = (spiDevicePrivateData_t*)device->privateData;
        if (devicePrivateData && devicePrivateData->spi_device_instance) {
            return (spi_master_status_e)privateData->spi_master_instance->Transmit(
                devicePrivateData->spi_device_instance, tx_data, length);
        }
    }
    return SPI_MASTER_STATUS_ERROR;
}

spi_master_status_e spiMasterReceive(spiMaster_t* master, spiDevice_t* device, uint8_t* rx_data,
                                     uint32_t length) {
    spiMasterPrivateData_t* privateData = (spiMasterPrivateData_t*)master->privateData;
    if (privateData && privateData->spi_master_instance) {
        spiDevicePrivateData_t* devicePrivateData = (spiDevicePrivateData_t*)device->privateData;
        if (devicePrivateData && devicePrivateData->spi_device_instance) {
            return (spi_master_status_e)privateData->spi_master_instance->Receive(
                devicePrivateData->spi_device_instance, rx_data, length);
        }
    }
    return SPI_MASTER_STATUS_ERROR;
}

spi_master_status_e spiMasterTransmitReceive(spiMaster_t* master, spiDevice_t* device,
                                             uint8_t* tx_data, uint8_t* rx_data, uint32_t length) {
    spiMasterPrivateData_t* privateData = (spiMasterPrivateData_t*)master->privateData;
    if (privateData && privateData->spi_master_instance) {
        spiDevicePrivateData_t* devicePrivateData = (spiDevicePrivateData_t*)device->privateData;
        if (devicePrivateData && devicePrivateData->spi_device_instance) {
            return (spi_master_status_e)privateData->spi_master_instance->TransmitReceive(
                devicePrivateData->spi_device_instance, tx_data, rx_data, length);
        }
    }
    return SPI_MASTER_STATUS_ERROR;
}

void spiMasterSetMode(spiMaster_t* master, spi_mode_e mode) {
    spiMasterPrivateData_t* privateData = (spiMasterPrivateData_t*)master->privateData;
    if (privateData && privateData->spi_master_instance) {
        privateData->spi_master_instance->SetMode((bsp::spi_mode_e)mode);
    }
}

void spiMasterSetAutoCS(spiMaster_t* master, bool auto_cs) {
    spiMasterPrivateData_t* privateData = (spiMasterPrivateData_t*)master->privateData;
    if (privateData && privateData->spi_master_instance) {
        privateData->spi_master_instance->SetAutoCS(auto_cs);
    }
}

// 工厂函数实现
spiPort_t* newSPIPort(SPI_HandleTypeDef* hspi, spi_mode_e mode) {
    if (!hspi)
        return nullptr;

    spiPort_t* instance = (spiPort_t*)malloc(sizeof(spiPort_t));
    if (!instance)
        return nullptr;

    memset(instance, 0, sizeof(spiPort_t));

    spiPrivateData_t* privateData = (spiPrivateData_t*)malloc(sizeof(spiPrivateData_t));
    if (!privateData) {
        free(instance);
        return nullptr;
    }

    bsp::spi_init_t init = {hspi, (bsp::spi_mode_e)mode};
    privateData->spi_instance = new bsp::SPI(init);
    if (!privateData->spi_instance) {
        free(privateData);
        free(instance);
        return nullptr;
    }

    // 设置VTable和私有数据
    instance->vTable = &spiVTable;
    instance->privateData = privateData;

    return instance;
}

spiDevice_t* newSPIDeviceViaMaster(spiMaster_t* master, void* cs) {
    spiMasterPrivateData_t* privateData = (spiMasterPrivateData_t*)master->privateData;
    if (privateData && privateData->spi_master_instance) {
        bsp::SPIDevice* bspDevice = privateData->spi_master_instance->NewDevice((bsp::GPIO*)cs);
        if (!bspDevice)
            return nullptr;

        spiDevice_t* device = (spiDevice_t*)malloc(sizeof(spiDevice_t));
        if (!device) {
            // todo: 实际应用中可能需要删除bspDevice以避免内存泄漏
            return nullptr;
        }

        memset(device, 0, sizeof(spiDevice_t));

        spiDevicePrivateData_t* devicePrivateData = (spiDevicePrivateData_t*)malloc(sizeof(spiDevicePrivateData_t));
        if (!devicePrivateData) {
            free(device);
            // todo: 实际应用中可能需要删除bspDevice以避免内存泄漏
            return nullptr;
        }

        devicePrivateData->spi_device_instance = bspDevice;
        device->privateData = devicePrivateData;
        return device;
    }
    return nullptr;
}

spiDevice_t* newSPIDevice(spiPort_t* spi, void* cs) {
    if (!spi)
        return nullptr;

    spiDevice_t* device = (spiDevice_t*)malloc(sizeof(spiDevice_t));
    if (!device)
        return nullptr;

    memset(device, 0, sizeof(spiDevice_t));

    spiDevicePrivateData_t* privateData =
        (spiDevicePrivateData_t*)malloc(sizeof(spiDevicePrivateData_t));
    if (!privateData) {
        free(device);
        return nullptr;
    }

    // todo: 获取SPI实例
    spiPrivateData_t* spiPrivateData = (spiPrivateData_t*)spi->privateData;
    if (!spiPrivateData || !spiPrivateData->spi_instance) {
        free(privateData);
        free(device);
        return nullptr;
    }

    bsp::spi_device_init_t init;
    init.spi = spiPrivateData->spi_instance;
    init.cs = (bsp::GPIO*)cs;
    privateData->spi_device_instance = new bsp::SPIDevice(init);
    if (!privateData->spi_device_instance) {
        free(privateData);
        free(device);
        return nullptr;
    }

    device->privateData = privateData;
    return device;
}

spiMaster_t* newSPIMaster(spiPort_t* spi) {
    if (!spi)
        return nullptr;

    spiMaster_t* master = (spiMaster_t*)malloc(sizeof(spiMaster_t));
    if (!master)
        return nullptr;

    memset(master, 0, sizeof(spiMaster_t));

    spiMasterPrivateData_t* privateData =
        (spiMasterPrivateData_t*)malloc(sizeof(spiMasterPrivateData_t));
    if (!privateData) {
        free(master);
        return nullptr;
    }

    // todo: 获取SPI实例
    spiPrivateData_t* spiPrivateData = (spiPrivateData_t*)spi->privateData;
    if (!spiPrivateData || !spiPrivateData->spi_instance) {
        free(privateData);
        free(master);
        return nullptr;
    }

    bsp::spi_master_init_t init;
    init.spi = spiPrivateData->spi_instance;
    privateData->spi_master_instance = new bsp::SPIMaster(init);
    if (!privateData->spi_master_instance) {
        free(privateData);
        free(master);
        return nullptr;
    }

    // 保存SPI实例的引用，以便后续使用
    privateData->spi_instance = spiPrivateData->spi_instance;

    master->privateData = privateData;
    return master;
}

// 释放函数实现
void deleteSPIPort(spiPort_t* instance) {
    if (!instance)
        return;

    spiPrivateData_t* privateData = (spiPrivateData_t*)instance->privateData;
    if (privateData) {
        if (privateData->spi_instance) {
            delete privateData->spi_instance;
        }
        free(privateData);
    }

    free(instance);
}

void deleteSPIDevice(spiDevice_t* device) {
    if (!device)
        return;

    spiDevicePrivateData_t* privateData = (spiDevicePrivateData_t*)device->privateData;
    if (privateData) {
        if (privateData->spi_device_instance) {
            delete privateData->spi_device_instance;
        }
        free(privateData);
    }

    free(device);
}

void deleteSPIMaster(spiMaster_t* master) {
    if (!master)
        return;

    spiMasterPrivateData_t* privateData = (spiMasterPrivateData_t*)master->privateData;
    if (privateData) {
        if (privateData->spi_master_instance) {
            delete privateData->spi_master_instance;
        }
        free(privateData);
    }

    free(master);
}
