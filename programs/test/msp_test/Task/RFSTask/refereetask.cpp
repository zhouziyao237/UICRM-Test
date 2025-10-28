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

#include "Device/bsp_Serial/UARTProtocol.h"
#include "Module/RFS/RefereeSerial.h"
#include "OSD.h"

// todo: text Module

// RMFC::UARTProtocol* UARTProtocol_ = nullptr;
// RMFC::RFS::RefereeProtocol* RefereeProtocol = nullptr;
// RMFC::RFS::RefereeProtConfig_t ProtConfig = {
//     .features = FEATURE_REF_PC_MSP,
//     .serialPortFunction = RMFC::FUNCTION_RefereeHost,
//     .baudRatesIndex = RMFC::REREFEE_BAUD_Index,
// };
//
// RMFC::UserInterface* UI = nullptr;

osThreadId_t RFSTaskHandle;

void InitRFS() {
    // Initialize Referee
    // serialPort_t* RefereeProt = FindRefereeProtocol(ProtConfig);
    // if (!RefereeProt) {
    //     print("Referee Init Failed\r\n");
    //     return;
    // }
    // RefereeProtocol = new RMFC::RFS::RefereeProtocol(RefereeProt);
    // UI = new RMFC::UserInterface(RefereeProt, RefereeProtocol);

    RMFC::RFS::RefereeSerialInit();
}

void UpdataRefereetask(void* args) {
    UNUSED(args);
    while (true) {
        // RMFC::RefereePort_t Refereetest = RMFC::RFS::RefereePort[0];
        // Refereetest.UI->SetID(1);
        // graphic_data_t lineGraphic;
        // Refereetest.UI->LineDraw(&lineGraphic, "LN1", UI_Graph_Add, 1, UI_Color_Purplish_red, 2,
        // 100, 100, 200, 200); Refereetest.UI->GraphRefresh(1, lineGraphic);

        RMFC::RFS::RefereeSerialProcess();
        osDelay(1);
    }
}