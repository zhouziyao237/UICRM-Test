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

#include "Common/config.h"
#include "Device/bsp_serial/DeviceSerial.h"
#include "Device/bsp_serial/bsp_Serial.h"
#include "Msp/MspSerial.h"
#include "MspProcessor.h"
#include "bsp_pg/bsp_pg.h"
#include "bsp_print.h"
#include "bsp_serial.h"
#include "cmsis_os2.h"

serialPort_t* serialmsp;
RMFC::MSP::MspProcessor _msp;
RMFC::MSP::MspMessage mspRequest;
RMFC::MSP::MspResponse mspResponse;

void RM_RTOS_Init() {
    // pgResetAll();

    initEEPROM();
    // ensureEEPROMStructureIsValid();
    readEEPROM();

    print_use_uart(&huart6, true, 912600);

    print("EEPROM start: 0x%08lX\r\n", (uint32_t)__config_start);
    print("EEPROM end: 0x%08lX\r\n", (uint32_t)__config_end);
    print("EEPROM size: %lu bytes\r\n", (uint32_t)(__config_end - __config_start));

    RMFC::serialInit(false, RMFC::SERIAL_PORT_NONE);
    RMFC::MSP::mspSerialInit();
    osDelay(100);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    while (true) {
        RMFC::MSP::mspSerialProcess(RMFC::MSP::MSP_EVALUATE_NON_MSP_DATA);
    }
}
