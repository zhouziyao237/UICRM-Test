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

PG_REGISTER_ARRAY_WITH_RESET_FN(SuperCap_t, MAX_PROFILE_COUNT, SuperCaps, PG_SuperCap_PROFILE, SuperCap_Version);

void resetCanBridges(SuperCap_t *SuperCaps) {
    RESET_CONFIG(SuperCap_t, SuperCaps,
        .can = nullptr,
        .Enable = true,
        .supercap_init =  {nullptr, 0x02E, 0x02F, 0x030},
        .MaxVoltage = 24.0f,
        .PowerTotal = 100.0f,
        .MaxChargePower = 150.0f,
        .MaxDischargePower = 250.0f,
        .PerferBuffer = 50.0f
        );
}

void pgResetFn_SuperCaps(void* SuperCaps, int size) {

    int count = size / sizeof(CanBridge_t);

    for (int i = 0; i < count && i < MAX_PROFILE_COUNT; i++) {
        SuperCap_t* SuperCap = static_cast<SuperCap_t*>(SuperCaps);
        resetCanBridges(&SuperCap[i]);
    }
}

void resetSuperCap(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        PG_RESET(SuperCaps);
    }
}

const SuperCap_t* getSuperCap(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        return SuperCaps(index);
    }
    return nullptr;
}

void copySuperCap(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < MAX_PROFILE_COUNT) && dstIndex != srcIndex) {
        memcpy(SuperCapsMutable(dstIndex), SuperCapsMutable(srcIndex), sizeof(SuperCap_t));
    }
}

void setCanBridge(uint8_t index, const SuperCap_t* SuperCaps) {
    if (index < MAX_PROFILE_COUNT) {
        SuperCap_t* SuperCap = SuperCapsMutable(index);
        *SuperCap = *SuperCaps;
    }
}