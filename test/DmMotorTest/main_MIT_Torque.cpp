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

#include "DMmotor.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "pid.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
static bsp::CAN* can2 = nullptr;
static driver::Motor6006* motor1 = nullptr;
static driver::Motor6006* motor2 = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart1, true, 912600);
    can2 = new bsp::CAN(&hcan2, false);
    motor1 = new driver::Motor6006(can2, 0x05, 0x01);
    motor1->SetTransmissionRatio(1);
    motor2 = new driver::Motor6006(can2, 0x06, 0x02);
    motor2->SetTransmissionRatio(1);

    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 8.0,        // 降低Kp，避免速度环输出过大
        .ki = 0,        // 降低Ki，减少积分饱和风险
        .kd = 2,        // 适当微分抑制震荡
        .max_out = 1200,  // 降低最大输出，留出安全余量 (todo: 输出范围缩放 100)
        .max_iout = 200,    // 限制积分项
        .deadband = 0, // 小死区提高灵敏度
        .A = 0.5 * PI,    // 缩小变速积分范围
        .B = 0.2 * PI,    // 降低启动死区
        .output_filtering_coefficient = 0.1, // 增强滤波平滑输出
        .derivative_filtering_coefficient = 0.0,
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };

    control::ConstrainedPID::PID_Init_t theta_pid_init = {
        .kp = 12,        // 降低Kp，避免位置环输出过大
        .ki = 0,        // 小积分消除静差
        .kd = 4,       // 小微分抑制震荡
        .max_out = 6 * 12.5f/2,  // 降低位置环最大输出，为速度环留出空间
        .max_iout = 0,    // 限制积分项
        .deadband = 0, // 提高灵敏度
        .A = 1.5 * PI,            // 关闭变速积分避免突变
        .B = 1 * PI,           // 关闭变速积分
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };

    motor1->ReInitPID(omega_pid_init, driver::DMMotorMITController::SPEED_LOOP_CONTROL);
    motor1->ReInitPID(theta_pid_init, driver::DMMotorMITController::ANGLE_LOOP_CONTROL);

    motor1->SetMode(driver::DMMotorMITController::ANGLE_LOOP_CONTROL |
                    driver::DMMotorMITController::SPEED_LOOP_CONTROL |
                    driver::DMMotorMITController::ABSOLUTE |
                    driver::DMMotorMITController::MOTOR_TORQUE_REVERSE);

    motor2->ReInitPID(omega_pid_init, driver::DMMotorMITController::SPEED_LOOP_CONTROL);
    motor2->ReInitPID(theta_pid_init, driver::DMMotorMITController::ANGLE_LOOP_CONTROL);

    motor2->SetMode(driver::DMMotorMITController::ANGLE_LOOP_CONTROL |
                    driver::DMMotorMITController::SPEED_LOOP_CONTROL |
                    driver::DMMotorMITController::ABSOLUTE |
                    driver::DMMotorMITController::MOTOR_TORQUE_REVERSE);

    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);
    print("test demo run");
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    motor1->Enable();
    motor2->Enable();
    osDelay(500);
    while (true) {
        set_cursor(0, 0);
        clear_screen();

        if (key.Read() == 0) {
            print("key");
            motor1->SetTarget(-0.5);  // 设置初始参数
            motor2->SetTarget(0.5);
        }

        motor1->PrintData();
        motor2->PrintData();
        osDelay(50);
    }
}
