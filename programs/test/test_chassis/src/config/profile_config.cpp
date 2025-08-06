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
// Created by Administrator on 2025/8/7.
//

#include "config.h"
#include "config_function.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(PROFILE_CONFIG_t, PROFILE_CONFIG_MAX_PROFILE_COUNT, PROFILECONFIG, PG_PROFILE_CONFIG_PROFILE, PROFILE_CONFIG_Version);

/* - PROFILE_CONFIG 配置 - */

void reestPROFILECONFIG(PROFILE_CONFIG_t* PROFILECONFIGS) {
    RESET_CONFIG(PROFILE_CONFIG_t, PROFILECONFIGS,
        .ControlModel_PROFILE = 0,
        .MotorPID_PROFILE = 0,
        .MotorId_PROFILE = 0,
        .CanBridge_PROFILE = 0,
        .RemoteDbus_PROFILE = 0,
        .RemoteSbus_PROFILE = 0,
        .SuperCap_PROFILE = 0,
    );
}

void pgResetFn_PROFILECONFIG(void* PROFILECONFIGS, int size) {

    int count = size / sizeof(PROFILE_CONFIG_t);

    for (int i = 0; i < count && i < PROFILE_CONFIG_MAX_PROFILE_COUNT; i++) {
        PROFILE_CONFIG_t* CONFIG = static_cast<PROFILE_CONFIG_t*>(PROFILECONFIGS);
        reestPROFILECONFIG(&CONFIG[i]);
    }
}

void resetindexPROFILECONFIG(uint8_t index) {
    if (index < PROFILE_CONFIG_MAX_PROFILE_COUNT) {
        PG_RESET(PROFILECONFIG);
    }
}

const PROFILE_CONFIG_t* getPROFILECONFIG(uint8_t index) {
    if (index < PROFILE_CONFIG_MAX_PROFILE_COUNT) {
        return PROFILECONFIG(index);
    }
    return nullptr;
}

void copyPROFILECONFIG(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < PROFILE_CONFIG_MAX_PROFILE_COUNT)) {
        memcpy(PROFILECONFIGMutable(dstIndex), PROFILECONFIG(srcIndex), sizeof(PROFILE_CONFIG_t));
    }
}

void setPROFILECONFIG(uint8_t index, const PROFILE_CONFIG_t* PROFILECONFIGS) {
    if (index < PROFILE_CONFIG_MAX_PROFILE_COUNT) {
        PROFILE_CONFIG_t* CONFIG = PROFILECONFIGMutable(index);
        *CONFIG = *PROFILECONFIGS;
    }
}