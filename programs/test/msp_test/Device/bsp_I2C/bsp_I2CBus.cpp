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

#include "bsp_I2CBus.h"
#include "bsp_i2c.h"
#include <stdlib.h>
#include <string.h>

// 私有数据结构
typedef struct {
    bsp::I2C* i2c_instance;
} i2cPrivateData_t;

// VTable函数实现
static bool i2cIsReadyImpl(i2cPort_t* instance, uint16_t id, uint32_t timeout) {
    i2cPrivateData_t* privateData = (i2cPrivateData_t*)instance->privateData;
    if (privateData && privateData->i2c_instance) {
        return privateData->i2c_instance->isReady(id, timeout);
    }
    return false;
}

static int i2cTransmitImpl(i2cPort_t* instance, uint16_t id, const uint8_t data[], uint16_t length) {
    i2cPrivateData_t* privateData = (i2cPrivateData_t*)instance->privateData;
    if (privateData && privateData->i2c_instance) {
        return privateData->i2c_instance->Transmit(id, data, length);
    }
    return -1;
}

static int i2cReceiveImpl(i2cPort_t* instance, uint16_t id, uint8_t* data, uint16_t length) {
    i2cPrivateData_t* privateData = (i2cPrivateData_t*)instance->privateData;
    if (privateData && privateData->i2c_instance) {
        return privateData->i2c_instance->Receive(id, data, length);
    }
    return -1;
}

static int i2cRead8Impl(i2cPort_t* instance, uint16_t id, uint16_t reg, uint8_t* data, uint16_t length) {
    i2cPrivateData_t* privateData = (i2cPrivateData_t*)instance->privateData;
    if (privateData && privateData->i2c_instance) {
        return privateData->i2c_instance->MemoryRead(id, reg, data, length);
    }
    return -1;
}

static int i2cRead16Impl(i2cPort_t* instance, uint16_t id, uint16_t reg, uint16_t* data, uint16_t length) {
    i2cPrivateData_t* privateData = (i2cPrivateData_t*)instance->privateData;
    if (privateData && privateData->i2c_instance) {
        return privateData->i2c_instance->MemoryRead(id, reg, data, length);
    }
    return -1;
}

static int i2cWrite8Impl(i2cPort_t* instance, uint16_t id, uint16_t reg, uint8_t* data, uint16_t length) {
    i2cPrivateData_t* privateData = (i2cPrivateData_t*)instance->privateData;
    if (privateData && privateData->i2c_instance) {
        return privateData->i2c_instance->MemoryWrite(id, reg, data, length);
    }
    return -1;
}

static int i2cWrite16Impl(i2cPort_t* instance, uint16_t id, uint16_t reg, uint16_t* data, uint16_t length) {
    i2cPrivateData_t* privateData = (i2cPrivateData_t*)instance->privateData;
    if (privateData && privateData->i2c_instance) {
        return privateData->i2c_instance->MemoryWrite(id, reg, data, length);
    }
    return -1;
}

static void i2cSetModeImpl(i2cPort_t* instance, bsp::i2c_mode_e mode) {
    i2cPrivateData_t* privateData = (i2cPrivateData_t*)instance->privateData;
    if (privateData && privateData->i2c_instance) {
        privateData->i2c_instance->SetMode((bsp::i2c_mode_e)mode);
    }
}

static bsp::i2c_mode_e i2cGetModeImpl(const i2cPort_t* instance) {
    i2cPrivateData_t* privateData = (i2cPrivateData_t*)instance->privateData;
    if (privateData && privateData->i2c_instance) {
        return (bsp::i2c_mode_e)privateData->i2c_instance->GetMode();
    }
    return bsp::I2C_MODE_BLOCKING;
}

static int i2cRegisterRxCallbackImpl(i2cPort_t* instance, uint32_t std_id, i2c_rx_callback_t callback, void* args) {
    i2cPrivateData_t* privateData = (i2cPrivateData_t*)instance->privateData;
    if (privateData && privateData->i2c_instance) {
        return privateData->i2c_instance->RegisterRxCallback(std_id, callback, args);
    }
    return -1;
}

// I2C VTable
static const struct i2cPortVTable i2cVTable = {
    i2cIsReadyImpl,
    i2cTransmitImpl,
    i2cReceiveImpl,
    i2cRead8Impl,
    i2cRead16Impl,
    i2cWrite8Impl,
    i2cWrite16Impl,
    i2cSetModeImpl,
    i2cGetModeImpl,
    i2cRegisterRxCallbackImpl
};

bool i2cIsReady(i2cPort_t* instance, uint16_t id, uint32_t timeout) {
    if (!instance || !instance->vTable || !instance->vTable->isReady)
        return false;
    return instance->vTable->isReady(instance, id, timeout);
}

int i2cTransmit(i2cPort_t* instance, uint16_t id, const uint8_t data[], uint16_t length) {
    if (!instance || !instance->vTable || !instance->vTable->transmit)
        return -1;
    return instance->vTable->transmit(instance, id, data, length);
}

int i2cReceive(i2cPort_t* instance, uint16_t id, uint8_t* data, uint16_t length) {
    if (!instance || !instance->vTable || !instance->vTable->receive)
        return -1;
    return instance->vTable->receive(instance, id, data, length);
}

int i2cRead8(i2cPort_t* instance, uint16_t id, uint16_t reg, uint8_t* data, uint16_t length) {
    if (!instance || !instance->vTable || !instance->vTable->memoryRead8)
        return -1;
    return instance->vTable->memoryRead8(instance, id, reg, data, length);
}

int i2cRead16(i2cPort_t* instance, uint16_t id, uint16_t reg, uint16_t* data, uint16_t length) {
    if (!instance || !instance->vTable || !instance->vTable->memoryRead16)
        return -1;
    return instance->vTable->memoryRead16(instance, id, reg, data, length);
}

int i2cWrite8(i2cPort_t* instance, uint16_t id, uint16_t reg, uint8_t* data, uint16_t length) {
    if (!instance || !instance->vTable || !instance->vTable->memoryWrite8)
        return -1;
    return instance->vTable->memoryWrite8(instance, id, reg, data, length);
}

int i2cWrite16(i2cPort_t* instance, uint16_t id, uint16_t reg, uint16_t* data, uint16_t length) {
    if (!instance || !instance->vTable || !instance->vTable->memoryWrite16)
        return -1;
    return instance->vTable->memoryWrite16(instance, id, reg, data, length);
}

void i2cSetMode(i2cPort_t* instance, bsp::i2c_mode_e mode) {
    if (!instance || !instance->vTable || !instance->vTable->setMode)
        return;
    instance->vTable->setMode(instance, mode);
}

bsp::i2c_mode_e i2cGetMode(const i2cPort_t* instance) {
    if (!instance || !instance->vTable || !instance->vTable->getMode)
        return bsp::I2C_MODE_BLOCKING;
    return instance->vTable->getMode(instance);
}

int i2cRegisterRxCallback(i2cPort_t* instance, uint32_t std_id, i2c_rx_callback_t callback, void* args) {
    if (!instance || !instance->vTable || !instance->vTable->registerRxCallback)
        return -1;
    return instance->vTable->registerRxCallback(instance, std_id, callback, args);
}

i2cPort_t* newI2CPort(I2C_HandleTypeDef* hi2c, bsp::i2c_mode_e mode) {
    if (!hi2c)
        return NULL;

    i2cPort_t* instance = (i2cPort_t*)malloc(sizeof(i2cPort_t));
    if (!instance)
        return NULL;

    memset(instance, 0, sizeof(i2cPort_t));

    i2cPrivateData_t* privateData = (i2cPrivateData_t*)malloc(sizeof(i2cPrivateData_t));
    if (!privateData) {
        free(instance);
        return NULL;
    }

    bsp::i2c_init_t init = {hi2c, (bsp::i2c_mode_e)mode};
    privateData->i2c_instance = new bsp::I2C(init);
    if (!privateData->i2c_instance) {
        free(privateData);
        free(instance);
        return NULL;
    }

    // 设置VTable和私有数据
    instance->vTable = &i2cVTable;
    instance->privateData = privateData;

    return instance;
}

// 释放I2C实例
void deleteI2CPort(i2cPort_t* instance) {
    if (!instance)
        return;

    i2cPrivateData_t* privateData = (i2cPrivateData_t*)instance->privateData;
    if (privateData) {
        if (privateData->i2c_instance) {
            delete privateData->i2c_instance;
        }
        free(privateData);
    }

    free(instance);
}
