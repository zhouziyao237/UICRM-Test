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
// Created by Administrator on 2025/11/19.
//

#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "pid.h"

#include "DMmotor/DMmotor.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
static bsp::CAN* can2 = nullptr;
static driver::DMMotor* motor = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart1, true, 912600);
    can2 = new bsp::CAN(&hcan2, true);
    motor = new driver::DMMotor(can2, 0x05, 0x01, driver::MIT);
    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);
    print("test demo run");
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    // motor->SetMITParams(0, 0.01, 12, 0.4, 10);  // 设置初始参数
    osDelay(500);

    motor->MotorEnable();
    while (true) {
        set_cursor(0, 0);
        clear_screen();

        if (key.Read() == 0) {
            motor->SetMITParams(0, 0, 0, 0, 4);
            // motor->SetMITParams(-1.3, 0.01, 12, 0.4, 10);  // 设置初始参数
        }


        motor->PrintData();
        // motor2->PrintData();
        osDelay(50);
    }
}

