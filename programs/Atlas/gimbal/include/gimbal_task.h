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

#pragma once

#include "main.h"
#include "sys.h"
#include "rc.h"
#include "gimbal.h"
#include "bsp_os.h"
#include "cmsis_os2.h"
#include "MotorCanBase.h"
#include "imu_task.h"

#include "minipc_task.h"
#include "referee_task.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "minipc_task.h"

// ----------------------------- Gimbal Task code -----------------------------

extern osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTaskAttribute = {
    .name = "GimbalTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0
};

// -------------------------- Gimbal Motor data code --------------------------

// --------------------------- Pitch Motor data code --------------------------
inline control::ConstrainedPID::PID_Init_t pitch_theta_pid_init = {
    .kp = 120,
    .ki = 0,
    .kd = 58,
    .max_out = 15 * PI,
    .max_iout = 0,
    .deadband = 0,                                 // 死区
    .A = 0,                                        // 变速积分所能达到的最大值为A+B
    .B = 0,                                        // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,           // 输出滤波系数
    .derivative_filtering_coefficient = 0,         // 微分滤波系数
    .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
};

inline control::ConstrainedPID::PID_Init_t pitch_omega_pid_init = {
    .kp = 120,
    .ki = 0,
    .kd = 2500,
    .max_out = 30000,
    .max_iout = 10000,
    .deadband = 0,                          // 死区
    .A = 1.5 * PI,                            // 变速积分所能达到的最大值为A+B
    .B = 1 * PI,                            // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,    // 输出滤波系数
    .derivative_filtering_coefficient = 0,  // 微分滤波系数
    .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
            control::ConstrainedPID::OutputFilter |         // 输出滤波
            control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
            control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
};

// ---------------------------- Yaw Motor data code --------------------------
inline control::ConstrainedPID::PID_Init_t yaw_theta_pid_init = {
    .kp = 20,
    .ki = 0,
    .kd = 0,
    .max_out = 6 * PI,
    .max_iout = 0,
    .deadband = 0,                                 // 死区
    .A = 0,                                        // 变速积分所能达到的最大值为A+B
    .B = 0,                                        // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,           // 输出滤波系数
    .derivative_filtering_coefficient = 0,         // 微分滤波系数
    .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
};

inline control::ConstrainedPID::PID_Init_t yaw_omega_pid_init = {
    .kp = 2500,
    .ki = 3,
    .kd = 0,
    .max_out = 30000,
    .max_iout = 10000,
    .deadband = 0,                                          // 死区
    .A = 1.5 * PI,                                          // 变速积分所能达到的最大值为A+B
    .B = 1 * PI,                                            // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,                    // 输出滤波系数
    .derivative_filtering_coefficient = 0,                  // 微分滤波系数
    .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
            control::ConstrainedPID::OutputFilter |         // 输出滤波
            control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
            control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
};

// ----------------------------- Gimbal data code -----------------------------

extern driver::MotorCANBase* pitch_motor;
extern driver::MotorCANBase* yaw_motor;
extern control::Gimbal* gimbal;
extern control::gimbal_data_t* gimbal_param;

typedef struct{
    float pitch_ratio;
    float yaw_ratio;
}GimbalRatio_t;
extern GimbalRatio_t GimbalRatio;

// ----------------------------- Gimbal Main code -----------------------------

void InitGimbal();
void GimbalRCUpdata();
uint8_t GimbalIsOnline();
void GimbalTask(void* arg);