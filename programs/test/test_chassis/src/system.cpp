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

#include "system.h"

#include "module_config/config.h"
#include "module_config/config_function.h"

bsp::CAN* Can_Motor = nullptr;
bsp::CAN* Can_Bridge = nullptr;

remote::DBUS* dbus = nullptr;
remote::SBUS* sbus = nullptr;

control::Chassis* chassis = nullptr;
communication::CanBridge* can_bridge = nullptr;
driver::SuperCap* super_cap = nullptr;

driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;

driver::MotorCANBase* motors[control::FourWheel::motor_num];

ChassisRemote_t chassis_remote;
ChassisControlMode_t Chissis_ControlMode = ModeNone;
system_model sys_state = SYSTEM_INIT;
SYS_CONFIG_t* SYS_PROFLIE_CONFIG = nullptr;

void sys_init() {
    SYS_PROFLIE_CONFIG = const_cast<SYS_CONFIG_t*>(getSYSCONFIG(PROFILE_CONFIG_PROFILE_COUNT));
    if (SYS_PROFLIE_CONFIG) {
        print("System Version %s\r\n", SYS_PROFLIE_CONFIG->sys_version);
        print("Profile Config OK\r\n");
        sys_state= SYSTEM_INIT;
    } else {
        resetindexSYSCONFIG(PROFILE_CONFIG_PROFILE_COUNT);
        sys_state = SYSTEM_ERRORCONTROL;
    }
}
