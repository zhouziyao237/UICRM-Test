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

//
// Created by Administrator on 2026/2/28.
//

#pragma once

#include "vtm.h"
#include "sys.h"
#include "RCKEY.h"
#include "bsp_thread.h"

// ----------------------------- RC Task code ----------------------------

const osThreadAttr_t RCTaskAttribute = {
    .name = "RCTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 512 * 2,
    .priority = (osPriority_t)osPriorityHigh,
    .tz_module = 0,
    .reserved = 0
};
extern osThreadId_t RCTaskHandle;

// ----------------------------- RC Data Code ---------------------------

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
    AUX27,
    AUX28,
    AUX29,
    AUX30,
    MOUSE_X_AXIS = AUX7,
    MOUSE_Y_AXIS = AUX8,
    MOUSE_WHEEL = AUX9,
    MOUSE_LEFT_BUTTON = AUX10,
    MOUSE_RIGHT_BUTTON = AUX11,
    MOUSE_MIDDLE_BUTTON = AUX12
} rc_alias_e;

#define MAX_SUPPORTED_RC_CHANNEL_COUNT 26
extern float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

// ----------------------------- VTM Data Code ------------------------------

#ifndef mouse_xy_max
#define mouse_xy_max 32767.0
#endif

#define MAXCHECK 660
extern remote::VTM* vtm;

// ----------------------------- RCKEY Data Code ----------------------------

extern rc_key_config_t rc_keys[RCKEYSNUM];

// ----------------------------- RC Main Code -------------------------------

void InitRC();
void RestartRC(int stage = 0);
void RCTask(void* arg);
uint8_t RCIsOnline();