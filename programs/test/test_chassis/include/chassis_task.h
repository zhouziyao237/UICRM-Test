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
// Created by Administrator on 2025/7/24.
//

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "MotorCanBase.h"
#include "chassis.h"
#include "config/config.h"

#define ratio 1.0f / 660.0f * 6 * PI

void InitSuperCap();
void InitMotor(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num]);
void InitMotor_PID(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num]);
void InitCanBridgeChassis(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num], void(*SuperCap(void)));
void InitRemoteDbusChassis(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num], void(*SuperCap(void)));
void InitRemoteSbusChassis(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num], void(*SuperCap(void)));

void ChassisInit(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num],
                 void(*SuperCap(void)), RemoteMode& Remote_mode, ChassisRemote_t &ChassisRemote);
void ChassisUpdate(RemoteMode& Remote_mode, ChassisRemote_t &ChassisRemote);

#endif //CHASSIS_TASK_H
