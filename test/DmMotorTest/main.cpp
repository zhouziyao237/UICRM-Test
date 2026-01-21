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
static driver::DMMotor* motor1 = nullptr;
static driver::DMMotor* motor2 = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart1, true, 912600);
    can2 = new bsp::CAN(&hcan2, true);
    motor1 = new driver::DMMotor(can2, 0x05, 0x01, driver::MIT);
    motor2 = new driver::DMMotor(can2, 0x06, 0x02, driver::MIT);
    // Snail need to be run at idle throttle for some
    // HAL_Delay(1000);
    print("test demo run");
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    motor1->MotorEnable();
    motor2->MotorEnable();
    motor1->SetMITParams(0, 0.01, 12, 0.4, 10);  // 设置初始参数
    motor2->SetMITParams(0, 0.01, 12, 0.4, 10);  // 设置初始参数
    osDelay(500);
    while (true) {
        set_cursor(0, 0);
        clear_screen();

        if (key.Read() == 0) {
            // motor1->SetMITParams(-1.3, 0.1, 8, 0.8, 10);
            motor2->SetMITParams(1.3, 0.01, 12, 0.4, 10);  // 设置初始参数
            motor1->SetMITParams(-1.3, 0.01, 12, 0.4, 10);  // 设置初始参数
        }


        motor1->PrintData();
        motor2->PrintData();
        osDelay(50);
    }
}

