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

#pragma once

#include "main.h"
#include "cmsis_os2.h"
#include "MotorCanBase.h"
#include "bsp_can.h"
#include "can_bridge.h"
#include "chassis.h"
#include "referee_task.h"
#include "rc.h"                     // 遥控器通道
#include "sys.h"                    // 系统模式
#include "utils.h"
#include "bsp_batteryvol.h"         // 电池电压测量

// ----------------------------- Chassis Task handle -----------------------------
extern osThreadId_t ChassisTaskHandle;
extern const osThreadAttr_t ChassisTaskAttribute;

// ----------------------------- CanBridge -----------------------------
extern communication::CanBridge* can_bridge;
extern control::ChassisCanBridgeSender* chassis;

// ----------------------------- Motion Data -----------------------------
extern float chassis_vx;   // X方向速度 (rad/s)
extern float chassis_vy;   // Y方向速度 (rad/s)
extern float chassis_vt;   // 旋转速度 (rad/s)

// ----------------------------- Battery Voltage -----------------------------
extern bsp::BatteryVol* battery_vol;

// ----------------------------- Chassis Parameters -----------------------------
extern const float chassis_max_xy_speed;
extern const float chassis_max_t_speed;

// ----------------------------- Functions -----------------------------
void InitChassis(void);          // 外部调用一次，初始化硬件和通信
void kill_chassis(void);
void ChassisTask(void* arg);