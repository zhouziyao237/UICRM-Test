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

PG_REGISTER_ARRAY_WITH_RESET_FN(MotorId_t, MAX_PROFILE_COUNT, MotorIds, PG_MotorId_PROFILE, PG_MotorId_PROFILE);

void resetMotorIds(MotorId_t *MotorIds) {
    RESET_CONFIG(MotorId_t, MotorIds,
        .fl_Id = 0x202,
        .fr_Id = 0x201,
        .bl_Id = 0x203,
        .br_Id = 0x204
        );
}

void pgResetFn_MotorIds(void* MotorIds, int size) {

    int count = size / sizeof(MotorId_t);

    for (int i = 0; i < count && i < MAX_PROFILE_COUNT; i++) {
        MotorId_t* IDS = static_cast<MotorId_t*>(MotorIds);
        resetMotorIds(&IDS[i]);
    }
}

// 重置特定索引的MotorId配置为默认值
void resetMotorId(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        PG_RESET(MotorIds);
    }
}

// 获取特定索引的MotorId配置
const MotorId_t* getMotorId(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        return MotorIds(index);
    }
    return nullptr;
}

// 复制MotorId配置
void copyMotorId(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < MAX_PROFILE_COUNT) && dstIndex != srcIndex) {
        memcpy(MotorIdsMutable(dstIndex), MotorIdsMutable(srcIndex), sizeof(MotorId_t));
    }
}

// 设置特定索引的MotorId参数
void setMotorId(uint8_t index, const MotorId_t* MotorIds) {
    if (index < MAX_PROFILE_COUNT) {
        MotorId_t* Id = MotorIdsMutable(index);
        *Id = *MotorIds;
    }
}