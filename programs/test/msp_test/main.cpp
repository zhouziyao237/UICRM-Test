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
#include "cmsis_os.h"
#include "bsp_serial.h"

Serial main_serial;

void RM_RTOS_Init() {
    main_serial.InitMainUart(&huart8, true, 912600);
    main_serial.print("uart test\n");
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    uint8_t* data = nullptr;
    while (true) {
        uint32_t len = main_serial.read(&data);
        if (len > 0) {
            main_serial.print("%s", data);
            data = nullptr;  // 清空数据指针，表示已处理完该数据
        }
        osDelay(1000);
    }
}