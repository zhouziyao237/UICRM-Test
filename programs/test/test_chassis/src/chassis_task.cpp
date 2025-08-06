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

#include "bsp_can.h"
#include "system.h"

void InitSuperCap(bsp::CAN* supercap_can) {
    if (const SuperCap_t* SuperCap = getSuperCap(SuperCap_PROFILE_COUNT)) {
        if (!SuperCap->Enable) {
            return;
        }
        driver::supercap_init_t supercap_init = {.can = supercap_can,
                                                 .tx_id = SuperCap->tx_id,
                                                 .tx_settings_id = SuperCap->tx_settings_id,
                                                 .rx_id = SuperCap->rx_id};
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
    } else {
        resetindexSuperCap(SuperCap_PROFILE_COUNT);
        system_state = SYSTEM_ERRORCONTROL;
    }
}

void InitMotor(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num]) {
    if (const MotorId_t* motorIds = getMotorId(MotorId_PROFILE_COUNT)) {
        Can_Motor = new bsp::CAN(motorIds->hcan, motorIds->is_master);
        fl_motor = new driver::Motor3508(Can_Motor, motorIds->fl_Id);
        fr_motor = new driver::Motor3508(Can_Motor, motorIds->fr_Id);
        bl_motor = new driver::Motor3508(Can_Motor, motorIds->bl_Id);
        br_motor = new driver::Motor3508(Can_Motor, motorIds->br_Id);
        driver::MotorCANBase* temp_motors[] = {fl_motor, fr_motor, bl_motor, br_motor};
        std::copy(temp_motors, temp_motors + 4, motor);
    } else {
        resetindexMotorId(MotorId_PROFILE_COUNT);
        system_state = SYSTEM_ERRORCONTROL;
        return;
    }
    return;
}

void InitMotor_PID(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num]) {
    if (const MotorPID_t* pid_params = getMotorPID(MotorPID_PROFILE_COUNT)) {

        control::ConstrainedPID::PID_Init_t pid_init = {
            .kp = pid_params->kp,
            .ki = pid_params->ki,
            .kd = pid_params->kd,
            .max_out = pid_params->max_out,
            .max_iout = pid_params->max_iout,
            .deadband = pid_params->deadband,  // 死区
            .A = pid_params->A,                // 变速积分所能达到的最大值为A+B
            .B = pid_params->B,                // 启动变速积分的死区
            .output_filtering_coefficient = pid_params->output_filtering_coefficient,  // 输出滤波系数
            .derivative_filtering_coefficient = pid_params->derivative_filtering_coefficient, // 微分滤波系数
            .mode = pid_params->mode,
        };

        constexpr int motor_count = control::FourWheel::motor_num;
        // 使用循环统一配置所有电机
        for (int i = 0; i < motor_count; i++) {
            motor[i]->ReInitPID(pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
            motor[i]->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
            motor[i]->SetTransmissionRatio(19);
        }
    } else {
        resetindexMotorPID(MotorPID_PROFILE_COUNT);
        system_state = SYSTEM_ERRORCONTROL;
    }
}

/* -- CanBridge --  */

void InitCanBridgeChassis(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num],
                          void(*SuperCap(void))) {
    if (const CanBridge_t* CanBridge = getCanBridge(CanBridge_PROFILE_COUNT)) {
        Can_Bridge = new bsp::CAN(CanBridge->hcan, CanBridge->is_master);
        can_bridge = new communication::CanBridge(Can_Bridge, CanBridge->CanBridge_RxId);

        motor[control::FourWheel::front_left] = fl_motor;
        motor[control::FourWheel::front_right] = fr_motor;
        motor[control::FourWheel::back_left] = bl_motor;
        motor[control::FourWheel::back_right] = br_motor;

        control::chassis_t chassis_data;
        chassis_data.motors = motor;
        chassis_data.model = CanBridge->Chassis_Model;
        chassis_data.omni_wheel_setup = CanBridge->OMNI_Wheel_setup_t;

        if (SuperCap != nullptr) {
            SuperCap();
            if (system_state == SYSTEM_ERRORCONTROL) {
                return;
            }
            chassis_data.has_super_capacitor = true;
            chassis_data.super_capacitor = super_cap;
        } else {
            chassis_data.has_super_capacitor = false;
        }

        chassis = new control::Chassis(chassis_data);
        chassis->SetMaxMotorSpeed(CanBridge->Max_Speed);
        chassis->CanBridgeSetTxId(CanBridge->CanBridge_TxId);
        can_bridge->RegisterRxCallback(CanBridge->CanBridge_XYWrapper,
                                       chassis->CanBridgeUpdateEventXYWrapper, chassis);
        can_bridge->RegisterRxCallback(CanBridge->CanBridge_TurnWrapper,
                                       chassis->CanBridgeUpdateEventTurnWrapper, chassis);
        can_bridge->RegisterRxCallback(CanBridge->CanBridge_PowerLimitWrapper,
                                       chassis->CanBridgeUpdateEventPowerLimitWrapper, chassis);
        can_bridge->RegisterRxCallback(CanBridge->CanBridge_CurrentPowerWrapper,
                                       chassis->CanBridgeUpdateEventCurrentPowerWrapper, chassis);
    } else {
        resetindexCanBridge(CanBridge_PROFILE_COUNT);
        system_state = SYSTEM_ERRORCONTROL;
    }
}

/* -- RemoteDbus --  */

void InitRemoteDbusChassis(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num],
                           void(*SuperCap(void))) {
    if (const RemoteDbus_t* RemoteDbus = getRemoteDbus(RemoteDbus_PROFILE_COUNT)) {
        motor[control::FourWheel::front_left] = fl_motor;
        motor[control::FourWheel::front_right] = fr_motor;
        motor[control::FourWheel::back_left] = bl_motor;
        motor[control::FourWheel::back_right] = br_motor;

        control::chassis_t chassis_data;
        chassis_data.motors = motor;
        chassis_data.model = RemoteDbus->Chassis_Model;
        chassis_data.omni_wheel_setup = RemoteDbus->OMNI_Wheel_setup_t;

        if (SuperCap != nullptr) {
            SuperCap();
            if (system_state == SYSTEM_ERRORCONTROL) {
                return;
            }
            chassis_data.has_super_capacitor = true;
            chassis_data.super_capacitor = super_cap;
        } else {
            chassis_data.has_super_capacitor = false;
        }

        dbus = new remote::DBUS(RemoteDbus->huart);
        chassis = new control::Chassis(chassis_data);
        chassis->SetMaxMotorSpeed(RemoteDbus->Max_Speed);

    } else {
        resetindexCanBridge(CanBridge_PROFILE_COUNT);
        system_state = SYSTEM_ERRORCONTROL;
    }
}

/* -- RemoteSbus --  */

void InitRemoteSbusChassis(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num],
                           void(*SuperCap(void))) {
    if (const RemoteSbus_t* RemoteSbus = getRemoteSbus(RemoteSbus_PROFILE_COUNT)) {
        motor[control::FourWheel::front_left] = fl_motor;
        motor[control::FourWheel::front_right] = fr_motor;
        motor[control::FourWheel::back_left] = bl_motor;
        motor[control::FourWheel::back_right] = br_motor;

        control::chassis_t chassis_data;
        chassis_data.motors = motor;
        chassis_data.model = RemoteSbus->Chassis_Model;
        chassis_data.omni_wheel_setup = RemoteSbus->OMNI_Wheel_setup_t;

        if (SuperCap != nullptr) {
            SuperCap();
            if (system_state == SYSTEM_ERRORCONTROL) {
                return;
            }
            chassis_data.has_super_capacitor = true;
            chassis_data.super_capacitor = super_cap;
        } else {
            chassis_data.has_super_capacitor = false;
        }

        sbus = new remote::SBUS(RemoteSbus->huart);
        chassis = new control::Chassis(chassis_data);
        chassis->SetMaxMotorSpeed(RemoteSbus->Max_Speed);

    } else {
        resetindexCanBridge(CanBridge_PROFILE_COUNT);
        system_state = SYSTEM_ERRORCONTROL;
    }
}

/* -- Chassis Main --  */

// 在 chassis_task.cpp 中的 ChassisInit 函数中
void ChassisInit(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num],
                 void(*SuperCap(void)), RemoteMode& Remote_mode, ChassisRemote_t& ChassisRemote) {
    if (const ControlModel_t* ControlModel = getControlModel(ControlModel_PROFILE_COUNT)) {
        Remote_mode = ControlModel->Remote_mode;
        switch (ControlModel->Remote_mode) {
            case RemoteModeDbus:
                if (const RemoteDbus_t* RemoteDbus = getRemoteDbus(RemoteDbus_PROFILE_COUNT)) {
                    ChassisRemote.remote_dbus = RemoteDbus;
                    InitRemoteDbusChassis(motor, SuperCap);
                } else {
                    resetindexRemoteDbus(RemoteDbus_PROFILE_COUNT);
                    system_state = SYSTEM_ERRORCONTROL;
                }
                break;
            case RemoteModeSbus:
                if (const RemoteSbus_t* RemoteSbus = getRemoteSbus(RemoteSbus_PROFILE_COUNT)) {
                    ChassisRemote.remote_sbus = RemoteSbus;
                    InitRemoteSbusChassis(motor, SuperCap);
                } else {
                    resetindexRemoteSbus(RemoteSbus_PROFILE_COUNT);
                    system_state = SYSTEM_ERRORCONTROL;
                }
                break;
            case RemoteModeCanBridge:
                // todo:在里面设置了CanBridge回调接收的ID不需要同步通道了
                InitCanBridgeChassis(motor, SuperCap);
                break;
            case RemoteModeNone:
                break;
            default:
                break;
        }
    } else {
        resetindexControlModel(ControlModel_PROFILE_COUNT);
        system_state = SYSTEM_ERRORCONTROL;
    }
}

void ChassisUpdate(RemoteMode& Remote_mode, ChassisRemote_t& ChassisRemote) {
    switch (Remote_mode) {
        case RemoteModeDbus: {
            const RemoteDbus_t* remote_dbus = ChassisRemote.remote_dbus;
            chassis->SetSpeed(dbus->channels[remote_dbus->XSpeedChannel] * ratio,
                              dbus->channels[remote_dbus->YSpeedChannel] * ratio,
                              dbus->channels[remote_dbus->TurnSpeedChannel] * ratio);
            chassis->SetPower(false, 30, 20, 60);
            chassis->Update();
        } break;
        case RemoteModeSbus: {
            const RemoteSbus_t* remote_sbus = ChassisRemote.remote_sbus;
            chassis->SetSpeed(sbus->channels[remote_sbus->XSpeedChannel] * ratio,
                              sbus->channels[remote_sbus->YSpeedChannel] * ratio,
                              sbus->channels[remote_sbus->TurnSpeedChannel] * ratio);
            chassis->SetPower(false, 30, 20, 60);
            chassis->Update();
        } break;
        case RemoteModeCanBridge:
            chassis->Update();
            break;
        case RemoteModeNone:
            break;
        default:
            // 处理未预期的枚举值
            break;
    }
}