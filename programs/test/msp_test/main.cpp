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

#include "MspProcessor.h"
#include "bsp_serial.h"
#include "cmsis_os.h"

Serial main_serial;
RMFC::Msp::MspProcessor _msp;
RMFC::Msp::MspMessage mspRequest;
RMFC::Msp::MspResponse mspResponse;

void RM_RTOS_Init() {
    // main_serial.InitMainUart(&huart8, true, 115200);
    main_serial.InitMainUsb();
    main_serial.print("uart test\n");
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    uint8_t* data = nullptr;
    while (true) {
        uint32_t len = main_serial.read(&data);
        if (len > 0) {
            char* c = (char*)data;
            for (uint32_t i = 0; i < len; i++) {
                _msp.process(*c, mspRequest, mspResponse, main_serial);
                c++;
            }
        }
    }
}