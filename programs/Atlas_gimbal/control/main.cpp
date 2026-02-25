// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
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
// Created by Administrator on 2026/2/24.
//

#include "main.h"

#include "remote_task.h"
#include "gyro_task.h"
#include "bsp_print.h"
#include "rx_vtm.h"

void RM_RTOS_Init()
{
    print_use_uart(&huart1, false, 912600);
    // 初始化遥控器系统
    InitRemote();
    // InitGyro();
}

void RM_RTOS_Threads_Init() {
    RemoteTaskHandle = osThreadNew(RemoteMainTask, nullptr, &remoteTaskAttribute);
    // GyroTaskHandle = osThreadNew(GyroMainTask, nullptr, &gyroTaskAttribute);
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);
    // 主控制循环
    while (true) {
        set_cursor(0, 0);
        clear_screen();

        print(
            "Accel: x: %.2f y: %.2f z: %.2f\r\n"
            "Gyro: x: %.2f y:%.2f z: %.2f"
            "Tempreture: %.2f",
            gyroData.gyroADCRaw[0], gyroData.gyroADCRaw[1], gyroData.gyroADCRaw[2],
            gyroData.gyroADC[0], gyroData.gyroADC[1], gyroData.gyroADC[2],
            gyroData.temperature);

        osDelay(100);
    }
}
