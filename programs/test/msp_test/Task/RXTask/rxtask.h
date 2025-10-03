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
// Created by Administrator on 2025/8/27.
//

#pragma once

#include "bsp_thread.h"
#include "bsp_tim.h"
#include "cmsis_os2.h"
#include "platform.h"

typedef enum { RX_STATE_CHECK, RX_STATE_MODES, RX_STATE_UPDATE, RX_STATE_COUNT } rxState_e;

bool InitRX();
bool UpdateRxMainInProgress(void);
void taskUpdateRxMain(void* args);
void rxProcessingLoop(void);

extern bsp::Timer* rxTimer;
extern bsp::InitTIMConfig RxTimConfig;

extern bsp::EventThread* RXcallbackthread_;
const osThreadAttr_t rx_thread_attr = {.name = "RxThread",
                                        .attr_bits = 0,
                                        .cb_mem = nullptr,  // 使用系统分配的线程控制块
                                        .cb_size = 0,       // 使用系统分配的线程控制块
                                        .stack_mem = nullptr,
                                        .stack_size = 512,
                                        .priority = (osPriority_t)osPriorityNormal,
                                        .tz_module = 0,
                                        .reserved = 0};