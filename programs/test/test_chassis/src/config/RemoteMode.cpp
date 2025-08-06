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
// Created by Administrator on 2025/8/5.
//

#include "config.h"

// 使用正确的宏注册ControlModel数组
PG_REGISTER_ARRAY_WITH_RESET_FN(ControlModel_t, MAX_PROFILE_COUNT, ControlModel, PG_ControlModel_PROFILE, ControlModel_Version);

/* - Control Model - */

void resatControlModel(ControlModel_t* ControlModels) {
    RESET_CONFIG(ControlModel_t, ControlModels,
        // todo: 暂时默认为Dbus模式
        .Remote_mode = RemoteModeDbus,
    );
}

void pgResetFn_ControlModel(void* ControlModels, int size) {
    int count = size / sizeof(ControlModel_t);
    for (int i = 0; i < count; i++) {
        ControlModel_t *ControlModel = static_cast<ControlModel_t*>(ControlModels);
        resatControlModel(&ControlModel[i]);
    }
}

void resetindexControlModel(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        PG_RESET(ControlModel);
    }
}

const ControlModel_t* getControlModel(uint8_t index) {
    if (index < MAX_PROFILE_COUNT) {
        return ControlModel(index);
    }
    return nullptr;
}

void copyControlModel(uint8_t dstIndex, uint8_t srcIndex) {
    if ((dstIndex < MAX_PROFILE_COUNT && srcIndex < MAX_PROFILE_COUNT) && dstIndex != srcIndex) {
        memcpy(ControlModelMutable(dstIndex), ControlModelMutable(srcIndex), sizeof(ControlModel_t));
    }
}

void setControlModel(uint8_t index, const ControlModel_t* ControlModels) {
    if (index < MAX_PROFILE_COUNT) {
        ControlModel_t* ControlModel = ControlModelMutable(index);
        *ControlModel = *ControlModels;
    }
}