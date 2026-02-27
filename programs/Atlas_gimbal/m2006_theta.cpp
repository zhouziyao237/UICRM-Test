/*###########################################################
# Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
#                                                          #
# This program is free software: you can redistribute it   #
# and/or modify it under the terms of the GNU General      #
# Public License as published by the Free Software         #
# Foundation, either version 3 of the License, or (at      #
# your option) any later version.                          #
#                                                          #
# This program is distributed in the hope that it will be  #
# useful, but WITHOUT ANY WARRANTY; without even           #
# the implied warranty of MERCHANTABILITY or FITNESS       #
# FOR A PARTICULAR PURPOSE.  See the GNU General           #
# Public License for more details.                         #
#                                                          #
# You should have received a copy of the GNU General       #
# Public License along with this program.  If not, see     #
# <https://www.gnu.org/licenses/>.                         #
###########################################################*/

#include "bsp_os.h"
#include "MotorCanBase.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "pid.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
static bsp::CAN* can1 = nullptr;
static driver::Motor2006* motor1 = nullptr;

struct StallDetection {

    struct CONFIG_s {
        // 阈值配置
        int16_t omega_thresh = 50;
        int16_t cmd_thresh = 5000;               // 忽略小电流指令（低于此值不检测）
        uint32_t stall_time_us = 500;            // 持续判定时间 (us)

        // 状态变量
        uint32_t stall_start = 0;
        bool stall_condition = false;
    };
    CONFIG_s* CONFIG;

    void SetConfig(CONFIG_s config) {
        CONFIG = new CONFIG_s(config);
    }

    // 更新堵转检测状态
    void Update(driver::Motor2006* motor) {
        int16_t cmd_current = motor->GetOutput();   // 控制电流指令 (LSB)
        int16_t omega = motor->GetOmega();

        if (abs(cmd_current) > CONFIG->cmd_thresh && abs(omega) < CONFIG->omega_thresh) {
            if (!CONFIG->stall_condition) {
                CONFIG->stall_condition = true;
                CONFIG->stall_start = bsp::GetHighresTickMilliSec();
            }
            if (CONFIG->stall_condition &&
                bsp::GetHighresTickMilliSec() - CONFIG->stall_start > CONFIG->stall_time_us) {
                motor->Disable();
                CONFIG->stall_start = 0;
                CONFIG->stall_condition = false;
                }
        } else if (abs(cmd_current) < CONFIG->cmd_thresh &&
                   bsp::GetHighresTickMilliSec() - CONFIG->stall_start > CONFIG->stall_time_us) {
            CONFIG->stall_start = 0;
            CONFIG->stall_condition = false;
                   }
    }

    // 重置状态
    void Reset() {
        CONFIG->stall_start = 0;
        CONFIG->stall_condition = false;
    }
};

static StallDetection stall_detector;

// 堵转检测回调函数（基于电流差）
void stall_detection_callback(void* instance) {
    driver::Motor2006* motor = static_cast<driver::Motor2006*>(instance);
    stall_detector.Update(motor);
}

void RM_RTOS_Init() {
    print_use_uart(&huart1, true, 912600);
    can1 = new bsp::CAN(&hcan1, true);
    motor1 = new driver::Motor2006(can1, 0x201);
    motor1->SetTransmissionRatio(36);
    control::ConstrainedPID::PID_Init_t theta_pid_init = {
        .kp = 20,
        .ki = 0,
        .kd = 0,
        .max_out = 15 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    motor1->ReInitPID(theta_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 1000,
        .ki = 1,
        .kd = 0,
        .max_out = 10000,
        .max_iout = 4000,
        .deadband = 0,                          // 死区
        .A = 3 * PI,                            // 变速积分所能达到的最大值为A+B
        .B = 2 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };
    motor1->ReInitPID(omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    motor1->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                    driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    motor1->RegisterDataUpdateCallback(stall_detection_callback, motor1);
    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    motor1->Enable();
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        if (key.Read() == 0) {
            osDelay(30);
            if (key.Read() == 1)
                continue;
            while (key.Read() == 0) {
                osDelay(30);
            }
            motor1->SetTarget(motor1->GetTarget() + 4 * PI/7);
            osDelay(200);
            motor1->SetTarget(motor1->GetTarget() + 4 * PI/7);
            }
        motor1->PrintData();
        print("输出电流: %d A\r\n", motor1->GetOutput());
        osDelay(20);
    }
}
