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
// Created by Administrator on 2026/3/3.
//

#pragma once


#include "main.h"
#include "rc.h"
#include "sys.h"
#include "bsp_os.h"
#include "chassis.h"
#include "cmsis_os2.h"
#include "MotorCanBase.h"

#include "gimbal_task.h"
#include "referee_task.h"
#include "minipc_task.h"
#include "imu_task.h"

// ----------------------------- Chassis Task code -----------------------------

extern osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTaskAttribute = {.name = "ChassisTask",
                                             .attr_bits = osThreadDetached,
                                             .cb_mem = nullptr,
                                             .cb_size = 0,
                                             .stack_mem = nullptr,
                                             .stack_size = 512 * 4,
                                             .priority = (osPriority_t)osPriorityAboveNormal,
                                             .tz_module = 0,
                                             .reserved = 0};

// --------------------------- Chassis CanBridge data code -------------------------

struct ChassisRegConfig_t {
    uint8_t tx_id;
    uint8_t rx_id;
    uint8_t xy_reg_id;           // X/Y 速度寄存器 ID
    uint8_t turn_on_reg_id;      // 旋转使能寄存器 ID
    uint8_t power_limit_reg_id;  // 功率限制寄存器 ID
    uint8_t current_power_reg_id; // 当前功率寄存器 ID
};

extern ChassisRegConfig_t ChassisRegConfig;
extern communication::CanBridge* can_bridge;
extern control::ChassisCanBridgeSender* chassis;

// --------------------------- Chassis data code -------------------------

struct ChassisPID {
    float kp;
    float ki;
    float kd;
    float max_out;
    float max_iout;
};

extern ChassisPID manual_theta_pid_init;

extern control::ConstrainedPID* manual_mode_pid;

struct ChassisData_t {
    float ratio = 1.0f / 660.0f * 10 * PI;
    float offset_yaw = 0;
    float spin_speed = 350;
    float yaw_pid_error = 0;
    float manual_mode_pid_output = 0;
    float current_speed_offset;
};

extern ChassisData_t ChassisData;

struct ChassisMotion_t {
    // 三角函数预计算值
    float sin_yaw = 0;
    float cos_yaw = 0;

    // 设定速度（原始值）
    float vx_set_org = 0;  // X 方向原始设定速度
    float vy_set_org = 0;  // Y 方向原始设定速度
    float offset_yaw = 0;

    // 设定速度（经过处理）
    float vx_set = 0;      // X 方向设定速度
    float vy_set = 0;      // Y 方向设定速度
    float vz_set = 0;      // 旋转方向设定速度

    // 实际反馈速度
    float chassis_vx = 0;  // X 方向实际速度
    float chassis_vy = 0;  // Y 方向实际速度
    float chassis_vz = 0;  // 旋转方向实际速度
};

extern ChassisMotion_t ChassisMotion;

enum VRAMP {
    CVRAMP_X = 0,
    CVRAMP_Y,
    CVRAMP_Z,
    CVRAMPNUM
};

enum chassis_key_edge_e {
    CFORWARD_EDGE = 0,   // 对应键盘 W
    CLEFT_EDGE,          // 对应键盘 A
    CBACKWARD_EDGE,      // 对应键盘 S
    CRIGHT_EDGE,         // 对应键盘 D
    CROTATE_LEFT_EDGE,   // 对应键盘 Q
    CROTATE_RIGHT_EDGE,  // 对应键盘 E

    CSPEED_UP_EDGE,   // 加速键（如 Shift）
    CSPEED_DOWN_EDGE,  // 减速键（如 Ctrl）
    CKEY_EDGE_NUM,
};

enum chassis_ch_edge_e{
    CFB_EDGE = 0,
    CLR_EDGE,
    CROTATE_EDGE,
    CCH_EDGE_NUM,
};

#define chassis_vx_max 660.0f / 2
#define chassis_vy_max 660.0f / 2
#define chassis_vz_max 660.0f / 2

extern float chassis_max[CVRAMPNUM];
extern RampSource* v_ramp[CVRAMPNUM];
extern BoolEdgeDetector* keyboard_edge[CKEY_EDGE_NUM];
extern BoolEdgeDetector* ch_edge[CCH_EDGE_NUM];

// ----------------------------- Chassis Main code -----------------------------

void InitChassis();
void ChassisRCUpdate();
uint8_t ChassisOnline();
void ChassisTask(void *arg);