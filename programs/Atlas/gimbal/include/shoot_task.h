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
// Created by Administrator on 2026/3/2.
//

#pragma once

#include "rc.h"
#include "sys.h"
#include "bsp_can.h"
#include "bsp_os.h"
#include "MotorCanBase.h"
#include "RCKEY.h"

#include "minipc_task.h"
#include "referee_task.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "minipc_task.h"

// ----------------------------- Shoot Task code -----------------------------

extern osThreadId_t ShootTaskHandle;
const osThreadAttr_t ShootTaskAttribute = {
    .name = "ShootTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0
};

// --------------------------- Shoot Motor data code --------------------------

// ---------------------------- FlyWheel Motor data code -----------------------
inline control::ConstrainedPID::PID_Init_t flywheel_pid_init = {
    .kp = 100,
    .ki = 1,
    .kd = 0,
    .max_out = 1000,
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

// ---------------------------- Feeding Motor data code ------------------------
inline control::ConstrainedPID::PID_Init_t steering_motor_theta_pid_init = {
    .kp = 30,
    .ki = 0,
    .kd = 0,
    .max_out = 5 * PI,
    .max_iout = 0,
    .deadband = 0,                                 // 死区
    .A = 0,                                        // 变速积分所能达到的最大值为A+B
    .B = 0,                                        // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,           // 输出滤波系数
    .derivative_filtering_coefficient = 0,         // 微分滤波系数
    .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
};
inline control::ConstrainedPID::PID_Init_t steering_motor_omega_pid_init = {
    .kp = 200,
    .ki = 0,
    .kd = 10000,
    .max_out = 10000,
    .max_iout = 4000,
    .deadband = 0,                          // 死区
    .A = 3 * PI,                            // 变速积分所能达到的最大值为A+B
    .B = 2 * PI,                            // 启动变速积分的死区
    .output_filtering_coefficient = 0.1,    // 输出滤波系数
    .derivative_filtering_coefficient = 0,  // 微分滤波系数
    .mode = control::ConstrainedPID::Integral_Limit |        // 积分限幅
            control::ConstrainedPID::OutputFilter |          // 输出滤波
            control::ConstrainedPID::Trapezoid_Intergral |   // 梯形积分
            control::ConstrainedPID::ChangingIntegralRate |  // 变速积分
            control::ConstrainedPID::ErrorHandle,            // 错误处理

};

// ----------------------------- Shoot data code -----------------------------

#define FW_SPEED 8 * PI
#define SINGLE_FEED_THETA 4 * PI/7
#define UNLOAD_FEED_THETA PI / 7

enum FrictionWheelMode_e {
    FW_MODE_OFF = 0,
    FW_MODE_ON = 1,
};

enum FrictionWheel_e {
    FW_LEFT = 0,
    FW_RIGHT = 1,
    FW_NUM,
};

extern FrictionWheelMode_e FWMode;

extern driver::MotorCANBase* FWMotor[FW_NUM];
extern driver::MotorCANBase* FeedingMotror;

enum shoot_edge_e {
    FEMODE_EDGE_CH = 0,
    FEMODE_EDGE_KEYBOARD,
    CLEAR_FEEDER_CH,
    CLEAR_FEEDER_KEYBOARD,
    SEDGENUM,
};

extern BoolEdgeDetector* shoot_edge[SEDGENUM];

struct StallDetection {

    typedef void (*StallCallback_t)(void* instance);

    struct CONFIG_s {
        // 阈值配置
        int16_t omega_thresh = 50;
        int16_t cmd_thresh = 5000;               // 忽略小电流指令（低于此值不检测）
        uint32_t stall_time_us = 500;            // 持续判定时间 (us)

        // 状态变量
        uint32_t stall_start = 0;
        bool stall_condition = false;
        StallCallback_t callback = nullptr;
        void* instance = nullptr;
    };
    CONFIG_s* CONFIG;

    void SetConfig(CONFIG_s config) {
        CONFIG = new CONFIG_s(config);
    }

    void SetCallback(StallCallback_t callback, void* instance) {
        CONFIG->callback = callback;
        CONFIG->instance = instance;
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
                CONFIG->callback(CONFIG->instance);
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

extern StallDetection stall_detector;


// --------------------------- Shoot Callback code ---------------------------

void trigger_callback(uint8_t channel, rc_key_mode_t mode);
void stall_detection_callback(void* instance);
void StallCallback(void* instance);

// ----------------------------- Shoot Main code -----------------------------

void InitShoot();
void unloadAmmo();
void ShootRCUpdata();
void ShootHeatControl();
uint8_t ShootIsOnline();
void ShootTask(void* arg);