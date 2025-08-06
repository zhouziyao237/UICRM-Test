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

PG_REGISTER_ARRAY_WITH_RESET_FN(MotorId_t, MAX_PROFILE_COUNT, MotorId, PG_MotorId_PROFILE, MotorId_Version);
PG_REGISTER_ARRAY_WITH_RESET_FN(CanBridge_t, MAX_PROFILE_COUNT, CanBridge, PG_CanBridge_PROFILE, CanBridge_Version);
PG_REGISTER_ARRAY_WITH_RESET_FN(RemoteDbus_t, MAX_PROFILE_COUNT, RemoteDbus, PG_RemoteDbus_PROFILE, RemoteDbus_Version);
PG_REGISTER_ARRAY_WITH_RESET_FN(RemoteSbus_t, MAX_PROFILE_COUNT, RemoteSbus, PG_RemoteSbus_PROFILE, RemoteSbus_Version);

void resetMotorId(MotorId_t *MotorIds) {
    RESET_CONFIG(MotorId_t, MotorIds,
        .hcan = &hcan2,
        .fl_Id = 0x202,
        .fr_Id = 0x201,
        .bl_Id = 0x203,
        .br_Id = 0x204
    );
}

void pgResetFn_MotorId(void* MotorIds, int size) {

    int count = size / sizeof(MotorId_t);

    for (int i = 0; i < count && i < MAX_PROFILE_COUNT; i++) {
        MotorId_t* IDS = static_cast<MotorId_t*>(MotorIds);
        resetMotorId(&IDS[i]);
    }
}

// 重置特定索引的MotorId配置为默认值
void resetindexMotorId(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        PG_RESET(MotorId);
    }
}

// 获取特定索引的MotorId配置
const MotorId_t* getMotorId(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        return MotorId(index);
    }
    return nullptr;
}

// 复制MotorId配置
void copyMotorId(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < MAX_PROFILE_COUNT) && dstIndex != srcIndex) {
        memcpy(MotorIdMutable(dstIndex), MotorIdMutable(srcIndex), sizeof(MotorId_t));
    }
}

// 设置特定索引的MotorId参数
void setMotorId(uint8_t index, const MotorId_t* MotorIds) {
    if (index < MAX_PROFILE_COUNT) {
        MotorId_t* Id = MotorIdMutable(index);
        *Id = *MotorIds;
    }
}

/*- CanBridge 配置 -*/

void resetCanBridge(CanBridge_t *CanBridges) {
    RESET_CONFIG(CanBridge_t, CanBridges,
        .hcan = &hcan1,
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

void pgResetFn_CanBridge(void* CanBridges, int size) {

    int count = size / sizeof(CanBridge_t);

    for (int i = 0; i < count && i < MAX_PROFILE_COUNT; i++) {
        CanBridge_t* CanBridge = static_cast<CanBridge_t*>(CanBridges);
        resetCanBridge(&CanBridge[i]);
    }
}

void resetindexCanBridge(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        PG_RESET(CanBridge);
    }
}

const CanBridge_t* getCanBridge(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        return CanBridge(index);
    }
    return nullptr;
}

void copyCanBridge(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < MAX_PROFILE_COUNT) && dstIndex != srcIndex) {
        memcpy(CanBridgeMutable(dstIndex), CanBridgeMutable(srcIndex), sizeof(CanBridge_t));
    }
}

void setCanBridge(uint8_t index, const CanBridge_t* CanBridges) {
    if (index < MAX_PROFILE_COUNT) {
        CanBridge_t* CanBridge = CanBridgeMutable(index);
        *CanBridge = *CanBridges;
    }
}

/* - RemoteDbus 配置 -*/

void resetRemoteDbus(RemoteDbus_t *RemoteDbuss) {
#if defined(DJI_Board_TypeA_general)
    RESET_CONFIG(RemoteDbus_t, RemoteDbuss,
        .huart = &huart1,
        .Enable_keyboard = false,
        .XSpeedChannel = 0,
        .YSpeedChannel = 1,
        .TurnSpeedChannel = 2,
        .Max_Speed = 2 * PI * 8,
        .Chassis_Model = control::CHASSIS_MECANUM_WHEEL,
        .OMNI_Wheel_setup_t = {45, 45, 45, 0.15f}
    );
#elif defined(DJI_Board_TypeC_general)
    RESET_CONFIG(RemoteDbus_t, RemoteDbuss,
        .huart = &huart3,
        .Enable_keyboard = false,
        .XSpeedChannel = 0,
        .YSpeedChannel = 1,
        .TurnSpeedChannel = 2,
        .Max_Speed = 2 * PI * 8,
        .Chassis_Model = control::CHASSIS_MECANUM_WHEEL,
        .OMNI_Wheel_setup_t = {45, 45, 45, 0.15f}
    );
#else
    #error "Please select a board type"
#endif
}

void pgResetFn_RemoteDbus(void* RemoteDbuss, int size) {
    int count = size / sizeof(RemoteDbus_t);
    for (int i = 0; i < count && i < MAX_PROFILE_COUNT; i++) {
        RemoteDbus_t* RemoteDbus = static_cast<RemoteDbus_t*>(RemoteDbuss);
        resetRemoteDbus(&RemoteDbus[i]);
    }
}

void resetindexRemoteDbus(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        PG_RESET(RemoteDbus);
    }
}

const RemoteDbus_t* getRemoteDbus(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        return RemoteDbus(index);
    }
    return nullptr;
}

void copyRemoteDbus(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < MAX_PROFILE_COUNT) && dstIndex != srcIndex) {
        memcpy(RemoteDbusMutable(dstIndex), RemoteDbusMutable(srcIndex), sizeof(RemoteDbus_t));
    }
}

void setRemoteDbus(uint8_t index, const RemoteDbus_t* RemoteDbuss) {
    if (index < MAX_PROFILE_COUNT) {
        RemoteDbus_t* RemoteDbus = RemoteDbusMutable(index);
        *RemoteDbus = *RemoteDbuss;
    }
}

/* - RemoteSbus 配置 -*/

void resetRemoteSbus(RemoteSbus_t *RemoteSbuss) {
#if defined(DJI_Board_TypeA_general)
    RESET_CONFIG(RemoteSbus_t, RemoteSbuss,
        .huart = &huart1,
        .Enable_keyboard = false,
        .XSpeedChannel = 0,
        .YSpeedChannel = 1,
        .TurnSpeedChannel = 2,
        .Max_Speed = 2 * PI * 8,
        .Chassis_Model = control::CHASSIS_MECANUM_WHEEL,
        .OMNI_Wheel_setup_t = {45, 45, 45, 0.15f}
    );
#elif defined(DJI_Board_TypeC_general)
    RESET_CONFIG(RemoteDbus_t, RemoteDbuss,
        .huart = &huart3,
        .Enable_keyboard = false,
        .XSpeedChannel = 0,
        .YSpeedChannel = 1,
        .TurnSpeedChannel = 2,
        .Max_Speed = 2 * PI * 8,
        .Chassis_Model = control::CHASSIS_MECANUM_WHEEL,
        .OMNI_Wheel_setup_t = {45, 45, 45, 0.15f}
    );
#else
#error "Please select a board type"
#endif
}

void pgResetFn_RemoteSbus(void* RemoteSbuss, int size) {
    int count = size / sizeof(RemoteSbus_t);
    for (int i = 0; i < count && i < MAX_PROFILE_COUNT; i++) {
        RemoteSbus_t* RemoteSbus = static_cast<RemoteSbus_t*>(RemoteSbuss);
        resetRemoteSbus(&RemoteSbus[i]);
    }
}

void resetindexRemoteSbus(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        PG_RESET(RemoteSbus);
    }
}

const RemoteSbus_t* getRemoteSbus(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        return RemoteSbus(index);
    }
    return nullptr;
}

void copyRemoteSbus(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < MAX_PROFILE_COUNT) && dstIndex != srcIndex) {
        memcpy(RemoteSbusMutable(dstIndex), RemoteSbusMutable(srcIndex), sizeof(RemoteSbus_t));
    }
}

void setRemoteSbus(uint8_t index, const RemoteSbus_t* RemoteSbuss) {
    if (index < MAX_PROFILE_COUNT) {
        RemoteSbus_t* RemoteSbus = RemoteSbusMutable(index);
        *RemoteSbus = *RemoteSbuss;
    }
}
