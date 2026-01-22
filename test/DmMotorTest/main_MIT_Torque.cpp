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

#include "DMmotor/DMmotor.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "pid.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
static bsp::CAN* can2 = nullptr;
static driver::DMMotorMITController* motor1 = nullptr;

void RM_RTOS_Init() {
    print_use_uart(&huart1, true, 912600);
    can2 = new bsp::CAN(&hcan2, true);
    motor1 = new driver::DMMotorMITController(can2, 0x08, 0x04);
    motor1->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 1.2,                                // 降低Kp，减速箱有放大效应
        .ki = 0.0,                                // 进一步降低，避免积分饱和
        .kd = 0.2,                                // 增加Kd，抑制减速箱间隙震荡
        .max_out = 10000,                         // 降低最大输出（针对输出轴）
        .max_iout = 2000,                         // 降低积分限幅
        .deadband = 0.01,                         // 增加小死区，避免微震荡
        .A = 0.5 * PI,                            // 进一步减小变速积分范围
        .B = 0.3 * PI,                            // 减小启动死区
        .output_filtering_coefficient = 0.3,      // 提高输出滤波
        .derivative_filtering_coefficient = 0.2,  // 提高微分滤波
        .mode = control::ConstrainedPID::Integral_Limit | control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate,
    };

    control::ConstrainedPID::PID_Init_t theta_pid_init = {
        .kp = 1.0,           // 角度环Kp可以稍高
        .ki = 0.0,           // 角度环通常不需要积分（用速度环积分）
        .kd = 0.2,           // 角度环需要更强微分
        .max_out = 5.0,      // 角度环输出是速度指令（rad/s）
        .max_iout = 1.0,
        .deadband = 0.005,   // 小角度死区
        .A = 0.2 * PI,       // 很小范围的变速积分
        .B = 0.1 * PI,
        .output_filtering_coefficient = 0.2,
        .derivative_filtering_coefficient = 0.15,
        .mode = control::ConstrainedPID::Integral_Limit |
                control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate,
    };

    motor1->ReInitPID(omega_pid_init, driver::DMMotorMITController::SPEED_LOOP_CONTROL);
    motor1->ReInitPID(theta_pid_init, driver::DMMotorMITController::ANGLE_LOOP_CONTROL);

    motor1->SetMode(driver::DMMotorMITController::ANGLE_LOOP_CONTROL |
                    driver::DMMotorMITController::SPEED_LOOP_CONTROL);
    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);
    print("test demo run");
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    motor1->Enable();
    osDelay(500);
    while (true) {
        set_cursor(0, 0);
        clear_screen();

        if (key.Read() == 0) {
            motor1->SetTarget(-0.2);  // 设置初始参数
        }

        motor1->PrintData();
        osDelay(50);
    }
}
