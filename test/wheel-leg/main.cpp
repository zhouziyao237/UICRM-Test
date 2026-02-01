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

#include "main.h"

#include "DMmotor.h"
#include "Kinematics.h"
#include "arm_math.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "pid.h"

#include "bsp_gpio.h"
#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

static Kinematics* kinematics = nullptr;
static bsp::CAN* can2 = nullptr;
static driver::DMMotorMITController* motoralpha = nullptr;
static driver::DMMotorMITController* motorbeta = nullptr;

void RM_RTOS_Init(void) {
    print_use_uart(&huart1, true, 921600);

    // 设置机构参数
    Kinematics::Model model;
    model.L1 = 130.0f;
    model.L2 = 210.0f;
    model.L3 = 130.0f;
    model.L4 = 210.0f;
    model.L5 = 90.0f;
    kinematics->setModel(model);

    can2 = new bsp::CAN(&hcan2, false);
    motoralpha = new driver::DMMotorMITController(can2, 0x05, 0x01);
    motoralpha->SetTransmissionRatio(1);

    motorbeta = new driver::DMMotorMITController(can2, 0x06, 0x02);
    motorbeta->SetTransmissionRatio(1);


    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 8.0,                            // 降低Kp，避免速度环输出过大
        .ki = 0,                              // 降低Ki，减少积分饱和风险
        .kd = 6,                              // 适当微分抑制震荡
        .max_out = 1200,                      // 降低最大输出，留出安全余量 (todo: 输出范围缩放 100)
        .max_iout = 200,                      // 限制积分项
        .deadband = 0,                        // 小死区提高灵敏度
        .A = 0.5 * PI,                        // 缩小变速积分范围
        .B = 0.2 * PI,                        // 降低启动死区
        .output_filtering_coefficient = 0.1,  // 增强滤波平滑输出
        .derivative_filtering_coefficient = 0.0,
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };

    control::ConstrainedPID::PID_Init_t theta_pid_init = {
        .kp = 12,                  // 降低Kp，避免位置环输出过大
        .ki = 0,                   // 小积分消除静差
        .kd = 8,                   // 小微分抑制震荡
        .max_out = 6 * 12.5f / 2,  // 降低位置环最大输出，为速度环留出空间
        .max_iout = 0,             // 限制积分项
        .deadband = 0,             // 提高灵敏度
        .A = 1.5 * PI,             // 关闭变速积分避免突变
        .B = 1 * PI,               // 关闭变速积分
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };

    motoralpha->ReInitPID(omega_pid_init, driver::DMMotorMITController::SPEED_LOOP_CONTROL);
    motoralpha->ReInitPID(theta_pid_init, driver::DMMotorMITController::ANGLE_LOOP_CONTROL);
    motoralpha->SetMode(driver::DMMotorMITController::ANGLE_LOOP_CONTROL |
                        driver::DMMotorMITController::SPEED_LOOP_CONTROL |
                        driver::DMMotorMITController::ABSOLUTE |
                        driver::DMMotorMITController::MOTOR_TORQUE_REVERSE);

    motorbeta->ReInitPID(omega_pid_init, driver::DMMotorMITController::SPEED_LOOP_CONTROL);
    motorbeta->ReInitPID(theta_pid_init, driver::DMMotorMITController::ANGLE_LOOP_CONTROL);
    motorbeta->SetMode(driver::DMMotorMITController::ANGLE_LOOP_CONTROL |
                       driver::DMMotorMITController::SPEED_LOOP_CONTROL |
                       driver::DMMotorMITController::ABSOLUTE |
                       driver::DMMotorMITController::MOTOR_TORQUE_REVERSE);

    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);
    print("test demo run");
}
void RM_RTOS_Threads_Init(void) {
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    motoralpha->Enable();
    motorbeta->Enable();
    osDelay(500);
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        if (key.Read() == 0) {
            // 逆运动学计算（根据足端坐标计算关节角度）
            // float x = 100.0f;  // 足端X坐标
            // float y = 50.0f;   // 足端Y坐标
            // Kinematics::IKResult ikResult = kinematics->inverseKinematics(x, y);
            //
            // // motoralpha->SetTarget(ikResult.alpha);
            // // motorbeta->SetTarget(ikResult.beta);
            // motoralpha->SetTarget(ikResult.alpha);
            // motorbeta->SetTarget(ikResult.beta);

            motoralpha->SetTarget(0 + motoralpha->GetAbsolutePosition());
            motorbeta->SetTarget(0);
        }

        motorbeta->PrintData();
        motoralpha->PrintData();
        osDelay(50);
    }
}