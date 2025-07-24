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

#include "chassis_task.h"

void InitMotor(bsp::CAN* can, MotorId_t motorIds){
    // 防止重复初始化导致内存泄漏
    static bool initialized = false;
    if (initialized) {
        return;
    }

    fl_motor = new driver::Motor3508(can, motorIds.fl_Id);
    fr_motor = new driver::Motor3508(can, motorIds.fr_Id);
    bl_motor = new driver::Motor3508(can, motorIds.bl_Id);
    br_motor = new driver::Motor3508(can, motorIds.br_Id);

    initialized = true;
}

void InitMotor_PID(control::ConstrainedPID::PID_Init_t pid_init) {

    // 将电机指针放入数组便于批量操作
    driver::MotorCANBase* motors[] = {fl_motor, fr_motor, bl_motor, br_motor};
    const size_t motor_count = sizeof(motors) / sizeof(motors[0]);

    // 使用循环统一配置所有电机
    for (size_t i = 0; i < motor_count; i++) {
        motors[i]->ReInitPID(pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
        motors[i]->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
        motors[i]->SetTransmissionRatio(19);
    }
}

void InitChassis(CanBridge_t CanBridge, void(*SuperCap(void))) {
    can_bridge = new communication::CanBridge(CanBridge.can, CanBridge.CanBridge_RxId);

    driver::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = CanBridge.Chassis_Model;

    if (SuperCap != nullptr) {
        SuperCap();
        chassis_data.has_super_capacitor = true;
        chassis_data.super_capacitor = super_cap;
    }else {
        chassis_data.has_super_capacitor = false;
    }

    chassis = new control::Chassis(chassis_data);
    chassis->SetMaxMotorSpeed(CanBridge.Max_Speed);
    chassis->CanBridgeSetTxId(CanBridge.CanBridge_TxId);
    can_bridge->RegisterRxCallback(CanBridge.CanBridge_XYWrapper, chassis->CanBridgeUpdateEventXYWrapper, chassis);
    can_bridge->RegisterRxCallback(CanBridge.CanBridge_TurnWrapper, chassis->CanBridgeUpdateEventTurnWrapper, chassis);
    can_bridge->RegisterRxCallback(CanBridge.CanBridge_PowerLimitWrapper, chassis->CanBridgeUpdateEventPowerLimitWrapper, chassis);
    can_bridge->RegisterRxCallback(CanBridge.CanBridge_CurrentPowerWrapper, chassis->CanBridgeUpdateEventCurrentPowerWrapper, chassis);
}

void InitSuperCap(SuperCap_t SuperCap) {
    if (!SuperCap.Enable) {
        return;
    }
    super_cap = new driver::SuperCap(SuperCap.supercap_init);
    super_cap->Disable();
    super_cap->TransmitSettings();
    super_cap->Enable();
    super_cap->TransmitSettings();
    super_cap->SetMaxVoltage(SuperCap.MaxVoltage);
    super_cap->SetPowerTotal(SuperCap.PowerTotal);
    super_cap->SetMaxChargePower(SuperCap.MaxChargePower);
    super_cap->SetMaxDischargePower(SuperCap.MaxDischargePower);
    super_cap->SetPerferBuffer(SuperCap.PerferBuffer);
}

void ChassisUpdate() {
    chassis->Update();
}