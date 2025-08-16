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

#pragma once

#include "MotorCanBase.h"
#include "bsp_can.h"
#include "config_eeprom/bsp_pg.h"
#include "chassis.h"

typedef struct {
    float kp = 2500.0f;  /// 比例系数
    float ki = 3.0f;     /// 积分系数
    float kd = 0.0f;     /// 微分系数

    float max_out = 30000;   /// 输出限幅
    float max_iout = 10000;  /// 积分输出限幅
    float deadband = 0;      /// 死区
    /// 变速积分
    float A = 3 * PI;                                              /// 变速积分所能达到的最大值为A+B
    float B = 2 * PI;                                              /// 启动变速积分的死区
    float output_filtering_coefficient = 0.1;                      /// 输出滤波系数
    float derivative_filtering_coefficient = 0;                    /// 微分滤波系数
    uint8_t mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                   control::ConstrainedPID::OutputFilter |         // 输出滤波
                   control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                   control::ConstrainedPID::ChangingIntegralRate;  // 变速积分
} __attribute__((packed)) MotorPID_t;

PG_DECLARE_ARRAY(MotorPID_t, MAX_PROFILE_COUNT, MotorPid);
