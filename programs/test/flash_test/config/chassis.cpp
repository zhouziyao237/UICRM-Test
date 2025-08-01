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
// Created by Administrator on 2025/8/1.
//

#include "config.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(MotorId_t, MAX_PROFILE_COUNT, MotorIds, PG_MotorId_PROFILE, MotorId_Version);
PG_REGISTER_ARRAY_WITH_RESET_FN(CanBridge_t, MAX_PROFILE_COUNT, CanBridges, PG_CanBridge_PROFILE, CanBridge_Version);

void resetMotorIds(MotorId_t *MotorIds) {
    RESET_CONFIG(MotorId_t, MotorIds,
        .fl_Id = 0x202,
        .fr_Id = 0x201,
        .bl_Id = 0x203,
        .br_Id = 0x204
        );
}

void resetCanBridges(CanBridge_t *CanBridges) {
    RESET_CONFIG(CanBridge_t, CanBridges,
        .can = nullptr,
        .CanBridge_RxId = 0x52,
        .CanBridge_TxId = 0x51,
        .CanBridge_XYWrapper = 0x70,
        .CanBridge_TurnWrapper = 0x71,
        .CanBridge_PowerLimitWrapper = 0x72,
        .CanBridge_CurrentPowerWrapper = 0x73,
        .Max_Speed = 2 * PI * 8,
        .Chassis_Model = control::CHASSIS_MECANUM_WHEEL,
        .OMNI_Wheel_setup_t = {45, 45, 45, 0.15f}
        );
}

void pgResetFn_MotorIds(void* MotorIds, int size) {

    int count = size / sizeof(MotorId_t);

    for (int i = 0; i < count && i < MAX_PROFILE_COUNT; i++) {
        MotorId_t* IDS = static_cast<MotorId_t*>(MotorIds);
        resetMotorIds(&IDS[i]);
    }
}

void pgResetFn_CanBridges(void* CanBridges, int size) {

    int count = size / sizeof(CanBridge_t);

    for (int i = 0; i < count && i < MAX_PROFILE_COUNT; i++) {
        CanBridge_t* CanBridge = static_cast<CanBridge_t*>(CanBridges);
        resetCanBridges(&CanBridge[i]);
    }
}

// 重置特定索引的MotorId配置为默认值
void resetMotorId(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        PG_RESET(MotorIds);
    }
}

void resetCanBridge(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        PG_RESET(CanBridges);
    }
}

// 获取特定索引的MotorId配置
const MotorId_t* getMotorId(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        return MotorIds(index);
    }
    return nullptr;
}

const CanBridge_t* getCanBridge(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        return CanBridges(index);
    }
    return nullptr;
}

// 复制MotorId配置
void copyMotorId(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < MAX_PROFILE_COUNT) && dstIndex != srcIndex) {
        memcpy(MotorIdsMutable(dstIndex), MotorIdsMutable(srcIndex), sizeof(MotorId_t));
    }
}

void copyCanBridge(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < MAX_PROFILE_COUNT) && dstIndex != srcIndex) {
        memcpy(CanBridgesMutable(dstIndex), CanBridgesMutable(srcIndex), sizeof(CanBridge_t));
    }
}

// 设置特定索引的MotorId参数
void setMotorId(uint8_t index, const MotorId_t* MotorIds) {
    if (index < MAX_PROFILE_COUNT) {
        MotorId_t* Id = MotorIdsMutable(index);
        *Id = *MotorIds;
    }
}

void setCanBridge(uint8_t index, const CanBridge_t* CanBridges) {
    if (index < MAX_PROFILE_COUNT) {
        CanBridge_t* CanBridge = CanBridgesMutable(index);
        *CanBridge = *CanBridges;
    }
}