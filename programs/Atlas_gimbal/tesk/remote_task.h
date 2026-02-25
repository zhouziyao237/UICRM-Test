// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
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

#pragma once

#include "config.h"
#include "cmsis_os2.h"
#include "usart.h"

//////////////////////////
/// RC channel aliases ///
//////////////////////////

typedef enum rc_alias {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
    AUX9,
    AUX10,
    AUX11,
    AUX12,
    AUX13,
    AUX14,
    AUX15,
    AUX16,
    AUX17,
    AUX18,
    AUX19,
    AUX20,
    AUX21,
    AUX22,
    AUX23,
    AUX24,
    AUX25,
    AUX26,
} rc_alias_e;

extern float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];  // interval [1000;2000]

typedef struct RemoteDeviceConfig_s {
    UART_HandleTypeDef uart;
    uint32_t baudrate;
} RemoteDeviceConfig_t;
extern RemoteDeviceConfig_t* RemoteDeviceConfig;

// ---------------------------------------- Model Main Code ----------------------------------------

/**
 * @brief 初始化遥控器系统
 */
void InitRemote();

void RemoteMainTask(void* arg);

extern osThreadId_t RemoteTaskHandle;
const osThreadAttr_t remoteTaskAttribute = {.name = "RemoteTask",
                                           .attr_bits = osThreadDetached,
                                           .cb_mem = nullptr,
                                           .cb_size = 0,
                                           .stack_mem = nullptr,
                                           .stack_size = 128 * 4,
                                           .priority = (osPriority_t)osPriorityHigh,
                                           .tz_module = 0,
                                           .reserved = 0};