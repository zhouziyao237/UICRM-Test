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

PG_REGISTER_ARRAY_WITH_RESET_FN(MotorPID_t, MAX_PROFILE_COUNT, MotorPids, PG_MotorPID_PROFILE, MotorPID_Version);

void resatMotorPids(MotorPID_t *MotorPids) {
    RESET_CONFIG(MotorPID_t, MotorPids,
        .kp = 2500.0f,
        .ki = 3.0f,
        .kd = 0.0f
    );
}

void pgResetFn_MotorPids(void *MotorPids, int size) {
    // 计算数组元素数量
    int count = size / sizeof(MotorPID_t);
    // 为每个配置文件设置默认值
    for (int i = 0; i < count && i < MAX_PROFILE_COUNT; i++) {
        MotorPID_t* PID = (MotorPID_t*)MotorPids;
        resatMotorPids(&PID[i]);
    }
}

// 重置特定索引的MotorPID配置为默认值
void resetMotorPID(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        PG_RESET(MotorPids);
    }
}

// 获取特定索引的MotorPID配置
const MotorPID_t* getMotorPID(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        return MotorPids(index);
    }
    return nullptr;
}

// 复制MotorPID配置
void copyMotorPID(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < MAX_PROFILE_COUNT) && dstIndex != srcIndex) {
        memcpy(MotorPidsMutable(dstIndex), MotorPidsMutable(srcIndex), sizeof(MotorPID_t));
    }
}

// 设置特定索引的MotorPID参数
void setMotorPID(uint8_t index, float kp, float ki, float kd) {
    if (index < MAX_PROFILE_COUNT) {
        MotorPID_t* pid = MotorPidsMutable(index);
        pid->kp = kp;
        pid->ki = ki;
        pid->kd = kd;
    }
}

