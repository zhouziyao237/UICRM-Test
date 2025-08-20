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

#include "main.h"

#include <RefereeProcessor.h>

#include "Common/config.h"
#include "Device/bsp_Serial/DeviceSerial.h"
#include "Device/bsp_Serial/bsp_Serial.h"
#include "Device/config/runtime_config.h"
#include "Msp/MspSerial.h"
#include "bsp_PG/bsp_pg.h"
#include "bsp_print.h"
#include "bsp_serial.h"
#include "cmsis_os2.h"

serialPort_t* RefereeSerial;
RMFC::RFS::RefereeProcessor RFSProcessor;
RMFC::RFS::RefereeMessage msg;
RMFC::RFS::RefereeResponse res;

void RM_RTOS_Init() {
    pgResetAll();

#ifdef USE_DEBUGUART
    print_use_uart(USE_DEBUGUART, false, 912600);
#endif

    print("EEPROM start: 0x%08lX\r\n", (uint32_t)__config_start);
    print("EEPROM end: 0x%08lX\r\n", (uint32_t)__config_end);
    print("EEPROM size: %lu bytes\r\n", (uint32_t)(__config_end - __config_start));

    initEEPROM();
    ensureEEPROMStructureIsValid();
    readEEPROM();

    RefereeSerial = newSerialPort(USEUART_6, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);

    // RMFC::serialInit(false, RMFC::SERIAL_PORT_NONE);
    // RMFC::MSP::mspSerialInit();
    osDelay(100);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    RMFC::RFS::rwstatus_e rwstatus = RMFC::RFS::Write_Status;
    uint8_t* data = nullptr;
    while (true) {
        // RMFC::MSP::mspEvaluateNonMspData_e evaluateMspData =
        //     ARMING_FLAG(ARMED) ? RMFC::MSP::MSP_SKIP_NON_MSP_DATA
        //                        : RMFC::MSP::MSP_EVALUATE_NON_MSP_DATA;
        // RMFC::MSP::mspSerialProcess(evaluateMspData);
        char* c = (char*)data;
        rwstatus = RMFC::RFS::Write_Status;
        uint32_t len = serialRead(RefereeSerial, &data);
        if (len > 0) {
            rwstatus = RMFC::RFS::Read_Status;
        }
        RFSProcessor.process(*c, rwstatus, msg, res, RefereeSerial);
    }
}
