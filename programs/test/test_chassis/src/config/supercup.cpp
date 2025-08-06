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
#include "can.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(SuperCap_t, MAX_PROFILE_COUNT, SuperCap, PG_SuperCap_PROFILE, SuperCap_Version);

void resetSuperCap(SuperCap_t *SuperCaps) {
    RESET_CONFIG(SuperCap_t, SuperCaps,
        .hcan = &hcan1,
        .Enable = true,
        .tx_id = 0x02E,
        .tx_settings_id = 0x02F,
        .rx_id = 0x030,
        .MaxVoltage = 24.0f,
        .PowerTotal = 100.0f,
        .MaxChargePower = 150.0f,
        .MaxDischargePower = 250.0f,
        .PerferBuffer = 50.0f
    );
}

void pgResetFn_SuperCap(void* SuperCaps, int size) {

    int count = size / sizeof(SuperCap_t);

    for (int i = 0; i < count && i < MAX_PROFILE_COUNT; i++) {
        SuperCap_t* SuperCap = static_cast<SuperCap_t*>(SuperCaps);
        resetSuperCap(&SuperCap[i]);
    }
}

// 重置特定索引的SuperCap配置为默认值
void resetindexSuperCap(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        PG_RESET(SuperCap);
    }
}

// 获取特定索引的SuperCap配置
const SuperCap_t* getSuperCap(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        return SuperCap(index);
    }
    return nullptr;
}

// 复制SuperCap配置
void copySuperCap(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < MAX_PROFILE_COUNT) && dstIndex != srcIndex) {
        memcpy(SuperCapMutable(dstIndex), SuperCapMutable(srcIndex), sizeof(SuperCap_t));
    }
}

// 设置特定索引的SuperCap参数
void setSuperCap(uint8_t index, const SuperCap_t* SuperCaps) {
    if (index < MAX_PROFILE_COUNT) {
        SuperCap_t* SuperCap = SuperCapMutable(index);
        *SuperCap = *SuperCaps;
    }
}