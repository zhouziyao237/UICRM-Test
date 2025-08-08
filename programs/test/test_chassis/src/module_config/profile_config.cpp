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

PG_REGISTER_ARRAY_WITH_RESET_FN(SYS_CONFIG_t, SYSTEM_CONFIG_MAX_PROFILE_COUNT, SYSCONFIG, PG_PROFILE_CONFIG_PROFILE, PROFILE_CONFIG_Version);

/* - PROFILE_CONFIG 配置 - */

void reestSYSCONFIG(SYS_CONFIG_t* SYSCONFIGS) {
    RESET_CONFIG(SYS_CONFIG_t, SYSCONFIGS,
        .ControlModel_PROFILE = 0,
        .MotorPID_PROFILE = 0,
        .MotorId_PROFILE = 0,
        .CanBridge_PROFILE = 0,
        .RemoteDbus_PROFILE = 0,
        .RemoteSbus_PROFILE = 0,
        .SuperCap_PROFILE = 0,
    );
}

void pgResetFn_SYSCONFIG(void* SYSCONFIGS, int size) {

    int count = size / sizeof(SYS_CONFIG_t);

    for (int i = 0; i < count && i < SYSTEM_CONFIG_MAX_PROFILE_COUNT; i++) {
        SYS_CONFIG_t* sysconfig = static_cast<SYS_CONFIG_t*>(SYSCONFIGS);
        reestSYSCONFIG(&sysconfig[i]);
    }
}

void resetindexSYSCONFIG(uint8_t index) {
    if (index < SYSTEM_CONFIG_MAX_PROFILE_COUNT) {
        PG_RESET(SYSCONFIG);
    }
}

const SYS_CONFIG_t* getSYSCONFIG(uint8_t index) {
    if (index < SYSTEM_CONFIG_MAX_PROFILE_COUNT) {
        return SYSCONFIG(index);
    }
    return nullptr;
}

void copySYSCONFIG(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < SYSTEM_CONFIG_MAX_PROFILE_COUNT)) {
        memcpy(SYSCONFIGMutable(dstIndex), SYSCONFIG(srcIndex), sizeof(SYS_CONFIG_t));
    }
}

void setSYSCONFIG(uint8_t index, const SYS_CONFIG_t* sys_config) {
    if (index < SYSTEM_CONFIG_MAX_PROFILE_COUNT) {
        SYS_CONFIG_t* CONFIG = SYSCONFIGMutable(index);
        *CONFIG = *sys_config;
    }
}