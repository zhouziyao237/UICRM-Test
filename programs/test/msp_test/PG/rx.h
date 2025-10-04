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
// Created by Administrator on 2025/8/23.
//

#pragma once

#include <stdint.h>

#include "Device/bsp_PG/bsp_pg.h"
#include "Module/RX/rx.h"
#include "Module/RX/rx_config.h"

#define GET_FRAME_ERR_LPF_FREQUENCY(period) (1 / (period / 10.0f))
#define FRAME_ERR_RESAMPLE_US 100000

namespace RMFC {
    namespace RX {
        typedef struct rxConfig_s {
            uint8_t serialrx_provider;  // 基于UART的接收机类型（0 = spek 10, 1 = spek 11, 2 =
            // sbus）。必须首先通过FEATURE_RX_SERIAL启用。
            uint8_t serialrx_inverted;  // 与默认设置相比，反转串行RX协议
            uint8_t halfDuplex;         // 允许rx在F4上以半双工模式运行，对F1和F3忽略。
            // uint8_t spektrum_bind_pin_override_ioTag;
            // uint8_t spektrum_bind_plug_ioTag;
            // uint8_t spektrum_sat_bind;            // Spektrum卫星接收机的绑定脉冲数
            // uint8_t spektrum_sat_bind_autoreset;  // 每当我们硬重启后是否重置（退出）绑定模式
            uint8_t rssi_channel;
            uint8_t rssi_scale;
            uint8_t rssi_invert;
            uint16_t midrc;                    // 一些无线电的中立点不以1500为中心。可以在这里更改
            uint16_t mincheck;                 // 最小rc末端值
            uint16_t maxcheck;                 // 最大rc末端值
            uint8_t fpvCamAngleDegrees;        // 摄像头角度，将被缩放为rc命令
            uint8_t airModeActivateThreshold;  // 气动模式被激活的油门设定点百分比
            uint16_t rx_min_usec;
            uint16_t rx_max_usec;
            uint8_t max_aux_channel;
            uint8_t rssi_src_frame_errors;            // 为真时使用rx协议中的帧丢失标志
            int8_t rssi_offset;                       // 在返回RSSI值之前应用的偏移量
            uint8_t rc_smoothing_mode;                // 基于滤波器的rc平滑是否开启
            uint8_t rc_smoothing_setpoint_cutoff;     // 设定点滤波器的截止频率（0 = 自动）
            uint8_t rc_smoothing_feedforward_cutoff;  // 前馈滤波器的截止频率（0 = 自动）
            uint8_t rc_smoothing_throttle_cutoff;     // 设定点滤波器的截止频率（0 = 自动）
            uint8_t rc_smoothing_debug_axis;  // 当debug_mode = RC_SMOOTHING时，记录为调试值的轴
            uint8_t rc_smoothing_auto_factor_rpy;       // 用于调整由自动截止频率计算确定的"平滑度"
            uint8_t rc_smoothing_auto_factor_throttle;  // 用于调整由自动截止频率计算确定的"平滑度"
            uint8_t rssi_src_frame_lpf_period;          // 源帧RSSI滤波器截止频率的周期（单位0.1秒）
            uint8_t rssi_smoothing;               // 平滑因子，用于减少rssi、rssiDbm和rsnr的跳动
            uint8_t srxl2_unit_id;                // Spektrum SRXL2 RX单元ID
            uint8_t srxl2_baud_fast;              // 选择Spektrum SRXL2快速波特率
            uint8_t sbus_baud_fast;               // 选择SBus快速波特率
            uint8_t dbus_baud_fast;               // 选择SBus快速波特率
            uint32_t msp_override_channels_mask;  // 当启用MSP覆盖模式时要覆盖的通道
            uint8_t crsf_use_negotiated_baud;     // 对CRSF V3使用协商的波特率
        } __attribute__((packed)) rxConfig_t;

        PG_DECLARE(rxConfig_t, rxConfig);

        // --------------------------------------- 分割线 ---------------------------------------

        typedef struct RPYTAConfig_s {
            uint8_t rcmap[RX_MAPPABLE_CHANNEL_COUNT];  // 无线电通道到内部RPYTA+顺序的映射
        } __attribute__((packed)) RPYTAConfig_t;

        PG_DECLARE(RPYTAConfig_t, RPYTAConfig);

    }  // namespace RX
}  // namespace RMFC