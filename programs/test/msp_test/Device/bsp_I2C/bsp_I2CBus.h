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

#include "i2c.h"
#include "bsp_i2c.h"

/* i2c callback function pointer */
typedef void (*i2c_rx_callback_t)(void* args);

typedef struct i2cPort_s i2cPort_t;

struct i2cPortVTable {
    bool (*isReady)(i2cPort_t* instance, uint16_t id, uint32_t timeout);
    int (*transmit)(i2cPort_t* instance, uint16_t id, const uint8_t data[], uint16_t length);
    int (*receive)(i2cPort_t* instance, uint16_t id, uint8_t* data, uint16_t length);
    int (*memoryRead8)(i2cPort_t* instance, uint16_t id, uint16_t reg, uint8_t* data, uint16_t length);
    int (*memoryRead16)(i2cPort_t* instance, uint16_t id, uint16_t reg, uint16_t* data, uint16_t length);
    int (*memoryWrite8)(i2cPort_t* instance, uint16_t id, uint16_t reg, uint8_t* data, uint16_t length);
    int (*memoryWrite16)(i2cPort_t* instance, uint16_t id, uint16_t reg, uint16_t* data, uint16_t length);
    void (*setMode)(i2cPort_t* instance, bsp::i2c_mode_e mode);
    bsp::i2c_mode_e (*getMode)(const i2cPort_t* instance);
    int (*registerRxCallback)(i2cPort_t* instance, uint32_t std_id, i2c_rx_callback_t callback, void* args);
};

typedef struct i2cPort_s {
    const struct i2cPortVTable* vTable;
    void* privateData;
} i2cPort_t;

i2cPort_t* newI2CPort(I2C_HandleTypeDef* hi2c, bsp::i2c_mode_e mode);

bool i2cIsReady(i2cPort_t* instance, uint16_t id, uint32_t timeout);
int i2cTransmit(i2cPort_t* instance, uint16_t id, const uint8_t data[], uint16_t length);
int i2cReceive(i2cPort_t* instance, uint16_t id, uint8_t* data, uint16_t length);
int i2cRead8(i2cPort_t* instance, uint16_t id, uint16_t reg, uint8_t* data, uint16_t length);
int i2cRead16(i2cPort_t* instance, uint16_t id, uint16_t reg, uint16_t* data, uint16_t length);
int i2cWrite8(i2cPort_t* instance, uint16_t id, uint16_t reg, uint8_t* data, uint16_t length);
int i2cWrite16(i2cPort_t* instance, uint16_t id, uint16_t reg, uint16_t* data, uint16_t length);
void i2cSetMode(i2cPort_t* instance, bsp::i2c_mode_e mode);
bsp::i2c_mode_e i2cGetMode(const i2cPort_t* instance);
int i2cRegisterRxCallback(i2cPort_t* instance, uint32_t std_id, i2c_rx_callback_t callback, void* args);

// 释放I2C实例
void deleteI2CPort(i2cPort_t* instance);
