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

#include "config.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(MotorPID_t, MAX_PROFILE_COUNT, MotorPid, PG_MotorPID_PROFILE, MotorPID_Version);

void resatMotorPid(MotorPID_t* MotorPids) {
    RESET_CONFIG(MotorPID_t, MotorPids,
        .kp = 2500.0f,
        .ki = 3.0f,
        .kd = 0.0f,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,
        .A = 3 * PI,
        .B = 2 * PI,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate   // 变速积分
    );

    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = MotorPids->kp,
        .ki = MotorPids->ki,
        .kd = MotorPids->kd,
        .max_out = MotorPids->max_out,
        .max_iout = MotorPids->max_iout,
        .deadband = MotorPids->deadband,  // 死区
        .A = MotorPids->A,                // 变速积分所能达到的最大值为A+B
        .B = MotorPids->B,                // 启动变速积分的死区
        .output_filtering_coefficient = MotorPids->output_filtering_coefficient,  // 输出滤波系数
        .derivative_filtering_coefficient = MotorPids->derivative_filtering_coefficient, // 微分滤波系数
        .mode = MotorPids->mode,
    };
    (void)omega_pid_init;
}

void pgResetFn_MotorPid(void* MotorPids, int size) {

    int count = size / sizeof(MotorPID_t);

    for (int i = 0; i < count && i < MAX_PROFILE_COUNT; i++) {
        MotorPID_t* PID = static_cast<MotorPID_t*>(MotorPids);
        resatMotorPid(&PID[i]);
    }
}

// 重置特定索引的MotorPID配置为默认值
void resetindexMotorPID(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        PG_RESET(MotorPid);
    }
}

// 获取特定索引的MotorPID配置
const MotorPID_t* getMotorPID(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        return MotorPid(index);
    }
    return nullptr;
}

// 复制MotorPID配置
void copyMotorPID(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < MAX_PROFILE_COUNT) && dstIndex != srcIndex) {
        memcpy(MotorPidMutable(dstIndex), MotorPidMutable(srcIndex), sizeof(MotorPID_t));
    }
}

// 设置特定索引的MotorPID参数
void setMotorPID(uint8_t index, const MotorPID_t* MotorPids) {
    if (index < MAX_PROFILE_COUNT) {
        MotorPID_t* pid = MotorPidMutable(index);
        *pid = *MotorPids;
    }
}
