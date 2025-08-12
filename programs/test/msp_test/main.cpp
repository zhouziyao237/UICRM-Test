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
#include "Device/serial/DeviceSerial.h"
#include "Device/serial/bsp_Serial.h"
#include "MspProcessor.h"
#include "bsp_serial.h"

serialPort_t* main_serial;
RMFC::Msp::MspProcessor _msp;
RMFC::Msp::MspMessage mspRequest;
RMFC::Msp::MspResponse mspResponse;

void RM_RTOS_Init() {
    // 使用新的Cleanflight风格串口接口打开USB串口
    main_serial = newSerialPortUSB();
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    while (true) {
        if (main_serial) {
            serialPrint(main_serial, "uart test\n");
        }
        // 检查是否有数据可读
        if (serialRxBytesWaiting(main_serial) > 0) {
            uint8_t ch = serialRead(main_serial);
            _msp.process(ch, mspRequest, mspResponse, *main_serial);
        }
    }
}
