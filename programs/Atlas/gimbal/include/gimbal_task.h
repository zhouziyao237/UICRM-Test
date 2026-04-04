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
#include "chassis_task.h"
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
    .kd = 580,
    .max_out = 15 * PI,
    .max_iout = 0,
    .deadband = 0,
    .A = 0,
    .B = 0,
    .output_filtering_coefficient = 0.1,
    .derivative_filtering_coefficient = 0,
    .mode = control::ConstrainedPID::OutputFilter
};

inline control::ConstrainedPID::PID_Init_t pitch_omega_pid_init = {
    .kp = 120,
    .ki = 0,
    .kd = 2500,
    .max_out = 30000,
    .max_iout = 10000,
    .deadband = 0,
    .A = 1.5 * PI,
    .B = 1 * PI,
    .output_filtering_coefficient = 0.1,
    .derivative_filtering_coefficient = 0,
    .mode = control::ConstrainedPID::Integral_Limit |
            control::ConstrainedPID::OutputFilter |
            control::ConstrainedPID::Trapezoid_Intergral |
            control::ConstrainedPID::ChangingIntegralRate,
};

// ---------------------------- Yaw Motor data code --------------------------
inline control::ConstrainedPID::PID_Init_t yaw_theta_pid_init = {
    .kp = 60,
    .ki = 0,
    .kd = 4000,
    .max_out = 6 * PI,
    .max_iout = 0,
    .deadband = 0,
    .A = 0,
    .B = 0,
    .output_filtering_coefficient = 0.1,
    .derivative_filtering_coefficient = 0,
    .mode = control::ConstrainedPID::OutputFilter
};

inline control::ConstrainedPID::PID_Init_t yaw_omega_pid_init = {
    .kp = 1500,
    .ki = 0,
    .kd = 0,
    .max_out = 16384,
    .max_iout = 2000,
    .deadband = 0,
    .A = 0.5 * PI,
    .B = 0.5 * PI,
    .output_filtering_coefficient = 0.1,
    .derivative_filtering_coefficient = 0,
    .mode = control::ConstrainedPID::Integral_Limit |
            control::ConstrainedPID::OutputFilter |
            control::ConstrainedPID::Trapezoid_Intergral |
            control::ConstrainedPID::ChangingIntegralRate,
};

// ----------------------------- Gimbal data code -----------------------------

extern driver::MotorCANBase* pitch_motor;
extern driver::MotorCANBase* yaw_motor;
extern control::Gimbal* gimbal;
extern control::gimbal_data_t* gimbal_param;

typedef struct {
    float pitch_ratio;
    float yaw_ratio;
} GimbalRatio_t;
extern GimbalRatio_t GimbalRatio;

// 统一世界坐标目标。所有模式都操作这一个目标，避免 manual/follow/spin 切换时参考系跳变。
typedef struct {
    float pitch;
    float yaw;
} GimbalWorldTarget_t;
extern GimbalWorldTarget_t GimbalWorldTarget;

// ----------------------------- Gimbal Main code -----------------------------

void InitGimbal();
void GimbalRCUpdata();
uint8_t GimbalIsOnline();
void GimbalTask(void* arg);