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
// Created by Administrator on 2025/7/28.
//

#pragma once

#include "MotorCanBase.h"
#include "chassis.h"
#include "bsp_can.h"
#include "config_eeprom/bsp_pg.h"

extern control::Chassis* chassis;
extern communication::CanBridge* can_bridge;
extern driver::SuperCap* super_cap;

extern driver::MotorCANBase* fl_motor;
extern driver::MotorCANBase* fr_motor;
extern driver::MotorCANBase* bl_motor;
extern driver::MotorCANBase* br_motor;

typedef struct {
  float kp = 2500.0f;
  float ki = 3.0f;
  float kd = 0.0f;
} __attribute__((packed)) MotorPID_t;

PG_DECLARE_ARRAY(MotorPID_t, MAX_PROFILE_COUNT, MotorPids);

// control::ConstrainedPID::PID_Init_t omega_pid_init = {
//   .kp = MotorPids->kp,
//   .ki = MotorPids->ki,
//   .kd = MotorPids->kd,
//   .max_out = 30000,
//   .max_iout = 10000,
//   .deadband = 0,                          // 死区
//   .A = 3 * PI,                            // 变速积分所能达到的最大值为A+B
//   .B = 2 * PI,                            // 启动变速积分的死区
//   .output_filtering_coefficient = 0.1,    // 输出滤波系数
//   .derivative_filtering_coefficient = 0,  // 微分滤波系数
//   .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
//           control::ConstrainedPID::OutputFilter |         // 输出滤波
//           control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
//           control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
// };
// (void)omega_pid_init;

typedef struct {
  uint16_t fl_Id = 0x202;
  uint16_t fr_Id = 0x201;
  uint16_t bl_Id = 0x203;
  uint16_t br_Id = 0x204;
} __attribute__((packed)) MotorId_t;

typedef struct {
  bsp::CAN* can = nullptr;
  uint16_t CanBridge_RxId = 0x52;
  uint16_t CanBridge_TxId = 0x51;
  uint16_t CanBridge_XYWrapper = 0x70;
  uint16_t CanBridge_TurnWrapper = 0x71;
  uint16_t CanBridge_PowerLimitWrapper = 0x72;
  uint16_t CanBridge_CurrentPowerWrapper = 0x73;
  float Max_Speed = 2 * PI * 8;
  control::chassis_model_t Chassis_Model = control::CHASSIS_MECANUM_WHEEL;
  control::OMNI_Wheel_setup_t OMNI_Wheel_setup_t;
} __attribute__((packed)) CanBridge_t;

typedef struct {
  bool Enable = true;
  driver::supercap_init_t supercap_init;
  float MaxVoltage = 24.0f;
  float PowerTotal = 100.0f;
  float MaxChargePower = 150.0f;
  float MaxDischargePower = 250.0f;
  float PerferBuffer = 50.0f;
} __attribute__((packed)) SuperCap_t;

void resatMotorPID(MotorPID_t *MotorPID);
void pgResetFn_MotorPids(void *MotorPids, int size);
void resetMotorPID(uint8_t index);
const MotorPID_t* getMotorPID(uint8_t index);
void copyMotorPID(uint8_t dstIndex, uint8_t srcIndex);
void setMotorPID(uint8_t index, float kp, float ki, float kd);
