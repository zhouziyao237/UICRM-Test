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
// Created by Administrator on 2025/8/20.
//

#pragma once

#include "Device/bsp_Serial/DeviceSerial.h"
#include "Module/RFS/RefereeProcessor.h"
#include "config/feature.h"

namespace RMFC {
    namespace RFS {

        typedef struct {
            features_e features;
            serialPortFunction_e serialPortFunction;
            baudrateFunctionIndex_e baudRatesIndex;
        } RefereeProtConfig_t;

        serialPort_t* FindRefereeProtocol(RefereeProtConfig_t ProtConfig);

    }  // namespace RFS
}  // namespace RMFC
