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
// Created by Administrator on 2025/8/30.
//

#include "refereetask.h"

#include "Module/RFS/RefereeSerial.h"

RMFC::RFS::RefereeProtocol* RefereeProtocol = nullptr;
RMFC::RFS::RefereeProtConfig_t ProtConfig = {
    .features = FEATURE_RFS_MSP,
    .serialPortFunction = RMFC::FUNCTION_RFS_MSP,
    .baudRatesIndex = RMFC::REREFEE_BAUD_Index,
};

void InitRFS() {
    // Initialize Referee
    RefereeProtocol = new RMFC::RFS::RefereeProtocol();
    bool ret = RefereeProtocol->RefereeInit(ProtConfig);
    if (!ret) {
        print("Referee Init Failed\r\n");
    }
}