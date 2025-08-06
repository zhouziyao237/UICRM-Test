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
// Created by Administrator on 2025/8/4.
//

#ifndef SYSTEM_H
#define SYSTEM_H

#include "MotorCanBase.h"
#include "bsp_can.h"
#include "chassis.h"
#include "dbus.h"
#include "sbus.h"
#include "config/config.h"

extern bsp::CAN* Can_Motor;
extern bsp::CAN* Can_Bridge;

extern remote::DBUS* dbus;
extern remote::SBUS* sbus;

extern control::Chassis* chassis;
extern communication::CanBridge* can_bridge;
extern driver::SuperCap* super_cap;

extern driver::MotorCANBase* fl_motor;
extern driver::MotorCANBase* fr_motor;
extern driver::MotorCANBase* bl_motor;
extern driver::MotorCANBase* br_motor;

extern driver::MotorCANBase* motors[control::FourWheel::motor_num];

enum system_model{
    SYSTEM_INIT = 0,
    SYSTEM_READCONFIG = 1,
    SYSTEM_READY = 2,
    SYSTEM_RUN = 3,
    SYSTEM_RESETCONFIG = 4,
    SYSTEM_SAVECONFIG = 5,
    SYSTEM_ERRORCONTROL = 6
};
extern system_model system_state;
extern RemoteMode remote_mode;
extern ChassisRemote_t chassis_remote;

#endif //SYSTEM_H
