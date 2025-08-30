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

#include "msptask.h"

#include "Device/config/runtime_config.h"
#include "Msp/MspSerial.h"
#include "cmsis_os2.h"

osThreadId_t MspTaskHandle;

void InitMSP() {
    // Initialize MSP
    // todo: but on test versin not have initActiveBoxIds
    // RMFC::MSP::mspInit();
    RMFC::MSP::mspSerialInit();
}

void UpdataMsptask(void* args) {
    UNUSED(args);
    while (true) {
#ifdef USE_CLI
        // in cli mode, all serial stuff goes to here. enter cli mode by sending #
        if (cliMode) {
            cliProcess();
            return;
        }
#endif
        RMFC::MSP::mspEvaluateNonMspData_e evaluateMspData =
            ARMING_FLAG(ARMED) ? RMFC::MSP::MSP_SKIP_NON_MSP_DATA
                               : RMFC::MSP::MSP_EVALUATE_NON_MSP_DATA;
        RMFC::MSP::mspSerialProcess(evaluateMspData);
        osDelay(1);
    }
}