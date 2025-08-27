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
// Created by Administrator on 2025/8/24.
//

# pragma once

// Extracted from rx/rx.c and rx/rx.h

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
#define DEFAULT_AUX_CHANNEL_COUNT       MAX_AUX_CHANNEL_COUNT
#else
#define DEFAULT_AUX_CHANNEL_COUNT       6
#endif

#define RX_MAPPABLE_CHANNEL_COUNT 8

#ifndef SERIALRX_PROVIDER
#define SERIALRX_PROVIDER 0
#endif

#define RX_MIN_USEC 885
#define RX_MAX_USEC 2115
#define RX_MID_USEC 1500

#define RSSI_ADC_DIVISOR (4096 / 1024)
#define RSSI_OFFSET_SCALING (1024 / 100.0f)

#define MAX_INVALID_PULSE_TIME_MS \
300  // 坏通道或接收链路丢失后的保持时间（毫秒）
         // 在最近的100毫秒倍数之前不会执行操作
#define PPM_AND_PWM_SAMPLE_COUNT 3

#define DELAY_20_MS (20 * 1000)      // 20毫秒（微秒）
#define DELAY_100_MS (100 * 1000)    // 100毫秒（微秒）
#define DELAY_1500_MS (1500 * 1000)  // 1.5秒（微秒）
#define SKIP_RC_SAMPLES_ON_RESUME 2  // 刷新2个采样以丢弃错误测量（时间无关）
