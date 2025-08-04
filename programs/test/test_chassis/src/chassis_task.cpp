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
#include "system.h"

bsp::CAN* Can_Motor = nullptr;
bsp::CAN* Can_Bridge = nullptr;

control::Chassis* chassis = nullptr;
communication::CanBridge* can_bridge = nullptr;
driver::SuperCap* super_cap = nullptr;

driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;

void InitMotor(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num]) {

    if (const MotorId_t* motorIds = getMotorId(MotorId_PROFILE_COUNT)) {
        Can_Motor = new bsp::CAN(motorIds->hcan, true);
        fl_motor = new driver::Motor3508(Can_Motor, motorIds->fl_Id);
        fr_motor = new driver::Motor3508(Can_Motor, motorIds->fr_Id);
        bl_motor = new driver::Motor3508(Can_Motor, motorIds->bl_Id);
        br_motor = new driver::Motor3508(Can_Motor, motorIds->br_Id);
        driver::MotorCANBase* temp_motors[] = {fl_motor, fr_motor, bl_motor, br_motor};
        std::copy(temp_motors, temp_motors + 4, motor);
    }else {
        resetMotorId(MotorId_PROFILE_COUNT);
        system_state = SYSTEM_ERRORCONTROL;
        return;
    }
    return;
}

void InitMotor_PID(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num]) {

    if (const MotorPID_t* pid_params = getMotorPID(MotorPID_PROFILE_COUNT)) {
        // 使用显式转换而不是reinterpret_cast来避免对齐警告
        control::ConstrainedPID::PID_Init_t pid_init;
        pid_init.kp = pid_params->kp;
        pid_init.ki = pid_params->ki;
        pid_init.kd = pid_params->kd;
        pid_init.max_out = pid_params->max_out;
        pid_init.max_iout = pid_params->max_iout;
        pid_init.deadband = pid_params->deadband;
        pid_init.A = pid_params->A;
        pid_init.B = pid_params->B;
        pid_init.output_filtering_coefficient = pid_params->output_filtering_coefficient;
        pid_init.derivative_filtering_coefficient = pid_params->derivative_filtering_coefficient;
        pid_init.mode = pid_params->mode;

        constexpr int motor_count = control::FourWheel::motor_num;
        // 使用循环统一配置所有电机
        for (int i = 0; i < motor_count; i++) {
            motor[i]->ReInitPID(pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
            motor[i]->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
            motor[i]->SetTransmissionRatio(19);
        }
    } else {
        resetMotorPID(MotorPID_PROFILE_COUNT);
        system_state = SYSTEM_ERRORCONTROL;
    }
}

void InitChassis(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num], void(*SuperCap(void))) {

    if (const CanBridge_t* CanBridge = getCanBridge(CanBridge_PROFILE_COUNT)) {
        Can_Bridge = new bsp::CAN(CanBridge->hcan, false);
        can_bridge = new communication::CanBridge(Can_Bridge, CanBridge->CanBridge_RxId);

        motor[control::FourWheel::front_left] = fl_motor;
        motor[control::FourWheel::front_right] = fr_motor;
        motor[control::FourWheel::back_left] = bl_motor;
        motor[control::FourWheel::back_right] = br_motor;

        control::chassis_t chassis_data;
        chassis_data.motors = motor;
        chassis_data.model = CanBridge->Chassis_Model;

        if (SuperCap != nullptr) {
            SuperCap();
            if (system_state == SYSTEM_ERRORCONTROL) {
                return;
            }
            chassis_data.has_super_capacitor = true;
            chassis_data.super_capacitor = super_cap;
        }else {
            chassis_data.has_super_capacitor = false;
        }

        chassis = new control::Chassis(chassis_data);
        chassis->SetMaxMotorSpeed(CanBridge->Max_Speed);
        chassis->CanBridgeSetTxId(CanBridge->CanBridge_TxId);
        can_bridge->RegisterRxCallback(CanBridge->CanBridge_XYWrapper, chassis->CanBridgeUpdateEventXYWrapper, chassis);
        can_bridge->RegisterRxCallback(CanBridge->CanBridge_TurnWrapper, chassis->CanBridgeUpdateEventTurnWrapper, chassis);
        can_bridge->RegisterRxCallback(CanBridge->CanBridge_PowerLimitWrapper, chassis->CanBridgeUpdateEventPowerLimitWrapper, chassis);
        can_bridge->RegisterRxCallback(CanBridge->CanBridge_CurrentPowerWrapper, chassis->CanBridgeUpdateEventCurrentPowerWrapper, chassis);
    } else {
        resetCanBridge(CanBridge_PROFILE_COUNT);
        system_state = SYSTEM_ERRORCONTROL;
    }
}

void InitSuperCap() {
    if (const SuperCap_t* SuperCap = getSuperCap(SuperCap_PROFILE_COUNT)) {
        if (!SuperCap->Enable) {
            return;
        }
        driver::supercap_init_t supercap_init = {
            .can = new bsp::CAN(SuperCap->hcan, false),
            .tx_id = SuperCap->tx_id,
            .tx_settings_id = SuperCap->tx_settings_id,
            .rx_id = SuperCap->rx_id
        };
        super_cap = new driver::SuperCap(supercap_init);
        super_cap->Disable();
        super_cap->TransmitSettings();
        super_cap->Enable();
        super_cap->TransmitSettings();
        super_cap->SetMaxVoltage(SuperCap->MaxVoltage);
        super_cap->SetPowerTotal(SuperCap->PowerTotal);
        super_cap->SetMaxChargePower(SuperCap->MaxChargePower);
        super_cap->SetMaxDischargePower(SuperCap->MaxDischargePower);
        super_cap->SetPerferBuffer(SuperCap->PerferBuffer);
    }else {
        resetSuperCap(SuperCap_PROFILE_COUNT);
        system_state = SYSTEM_ERRORCONTROL;
    }
}

void ChassisUpdate() {
    chassis->Update();
}