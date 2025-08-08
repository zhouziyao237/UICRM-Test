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
#include "module_config/config.h"
#include "module_config/config_function.h"
#include "system.h"

void InitSuperCap(bsp::CAN* supercap_can) {
    SuperCap_CONFIG_DATA = const_cast<SuperCap_t*>(getSuperCap(SYS_PROFLIE_CONFIG_DATA->SuperCap_PROFILE));
    if (SuperCap_CONFIG_DATA) {
        if (!SuperCap_CONFIG_DATA->Enable) {
            return;
        }
        driver::supercap_init_t supercap_init = {.can = supercap_can,
                                                 .tx_id = SuperCap_CONFIG_DATA->tx_id,
                                                 .tx_settings_id = SuperCap_CONFIG_DATA->tx_settings_id,
                                                 .rx_id = SuperCap_CONFIG_DATA->rx_id};
        super_cap = new driver::SuperCap(supercap_init);
        super_cap->Disable();
        super_cap->TransmitSettings();
        super_cap->Enable();
        super_cap->TransmitSettings();
        super_cap->SetMaxVoltage(SuperCap_CONFIG_DATA->MaxVoltage);
        super_cap->SetPowerTotal(SuperCap_CONFIG_DATA->PowerTotal);
        super_cap->SetMaxChargePower(SuperCap_CONFIG_DATA->MaxChargePower);
        super_cap->SetMaxDischargePower(SuperCap_CONFIG_DATA->MaxDischargePower);
        super_cap->SetPerferBuffer(SuperCap_CONFIG_DATA->PerferBuffer);
    } else {
        resetindexSuperCap(SYS_PROFLIE_CONFIG_DATA->SuperCap_PROFILE);
        sys_state = SYSTEM_ERRORCONTROL;
    }
}

void InitMotor(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num]) {
    MotorId_CONFIG_DATA = const_cast<MotorId_t*>(getMotorId(SYS_PROFLIE_CONFIG_DATA->MotorId_PROFILE));
    if (MotorId_CONFIG_DATA) {
        Can_Motor = new bsp::CAN(MotorId_CONFIG_DATA->hcan, MotorId_CONFIG_DATA->is_master);
        fl_motor = new driver::Motor3508(Can_Motor, MotorId_CONFIG_DATA->fl_Id);
        fr_motor = new driver::Motor3508(Can_Motor, MotorId_CONFIG_DATA->fr_Id);
        bl_motor = new driver::Motor3508(Can_Motor, MotorId_CONFIG_DATA->bl_Id);
        br_motor = new driver::Motor3508(Can_Motor, MotorId_CONFIG_DATA->br_Id);
        driver::MotorCANBase* temp_motors[] = {fl_motor, fr_motor, bl_motor, br_motor};
        std::copy(temp_motors, temp_motors + 4, motor);
    } else {
        resetindexMotorId(SYS_PROFLIE_CONFIG_DATA->MotorId_PROFILE);
        sys_state = SYSTEM_ERRORCONTROL;
        return;
    }
    return;
}

void InitMotor_PID(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num]) {
    MotorPID_t* pid_params_CONFIG_DATA = const_cast<MotorPID_t*>(getMotorPID(SYS_PROFLIE_CONFIG_DATA->MotorPID_PROFILE));
    if (pid_params_CONFIG_DATA) {

        control::ConstrainedPID::PID_Init_t pid_init = {
            .kp = pid_params_CONFIG_DATA->kp,
            .ki = pid_params_CONFIG_DATA->ki,
            .kd = pid_params_CONFIG_DATA->kd,
            .max_out = pid_params_CONFIG_DATA->max_out,
            .max_iout = pid_params_CONFIG_DATA->max_iout,
            .deadband = pid_params_CONFIG_DATA->deadband,  // 死区
            .A = pid_params_CONFIG_DATA->A,                // 变速积分所能达到的最大值为A+B
            .B = pid_params_CONFIG_DATA->B,                // 启动变速积分的死区
            .output_filtering_coefficient = pid_params_CONFIG_DATA->output_filtering_coefficient,  // 输出滤波系数
            .derivative_filtering_coefficient = pid_params_CONFIG_DATA->derivative_filtering_coefficient, // 微分滤波系数
            .mode = pid_params_CONFIG_DATA->mode,
        };

        // 使用循环统一配置所有电机
        for (int i = 0; i < control::FourWheel::motor_num; i++) {
            motor[i]->ReInitPID(pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
            motor[i]->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
            motor[i]->SetTransmissionRatio(19);
        }
    } else {
        resetindexMotorPID(SYS_PROFLIE_CONFIG_DATA->MotorPID_PROFILE);
        sys_state = SYSTEM_ERRORCONTROL;
    }
}

/* -- CanBridge --  */

void InitCanBridgeChassis(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num],
                          void(*SuperCap(void))) {
    CanBridge_t* CanBridge_CONFIG_DATA = const_cast<CanBridge_t*>(getCanBridge(SYS_PROFLIE_CONFIG_DATA->CanBridge_PROFILE));
    if (CanBridge_CONFIG_DATA) {
        Can_Bridge = new bsp::CAN(CanBridge_CONFIG_DATA->hcan, CanBridge_CONFIG_DATA->is_master);
        can_bridge = new communication::CanBridge(Can_Bridge, CanBridge_CONFIG_DATA->CanBridge_RxId);

        motor[control::FourWheel::front_left] = fl_motor;
        motor[control::FourWheel::front_right] = fr_motor;
        motor[control::FourWheel::back_left] = bl_motor;
        motor[control::FourWheel::back_right] = br_motor;

        control::chassis_t chassis_data;
        chassis_data.motors = motor;
        chassis_data.model = CanBridge_CONFIG_DATA->Chassis_Model;
        chassis_data.omni_wheel_setup = CanBridge_CONFIG_DATA->OMNI_Wheel_setup_t;

        if (SuperCap != nullptr) {
            SuperCap();
            if (sys_state == SYSTEM_ERRORCONTROL) {
                return;
            }
            chassis_data.has_super_capacitor = true;
            chassis_data.super_capacitor = super_cap;
        } else {
            chassis_data.has_super_capacitor = false;
        }

        chassis = new control::Chassis(chassis_data);
        chassis->SetMaxMotorSpeed(CanBridge_CONFIG_DATA->Max_Speed);
        chassis->CanBridgeSetTxId(CanBridge_CONFIG_DATA->CanBridge_TxId);
        can_bridge->RegisterRxCallback(CanBridge_CONFIG_DATA->CanBridge_XYWrapper,
                                       chassis->CanBridgeUpdateEventXYWrapper, chassis);
        can_bridge->RegisterRxCallback(CanBridge_CONFIG_DATA->CanBridge_TurnWrapper,
                                       chassis->CanBridgeUpdateEventTurnWrapper, chassis);
        can_bridge->RegisterRxCallback(CanBridge_CONFIG_DATA->CanBridge_PowerLimitWrapper,
                                       chassis->CanBridgeUpdateEventPowerLimitWrapper, chassis);
        can_bridge->RegisterRxCallback(CanBridge_CONFIG_DATA->CanBridge_CurrentPowerWrapper,
                                       chassis->CanBridgeUpdateEventCurrentPowerWrapper, chassis);
    } else {
        resetindexCanBridge(SYS_PROFLIE_CONFIG_DATA->CanBridge_PROFILE);
        sys_state = SYSTEM_ERRORCONTROL;
    }
}

/* -- RemoteDbus --  */

void InitRemoteDbusChassis(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num],
                           void(*SuperCap(void))) {
    RemoteDbus_t* RemoteDbus_CONFIG_DATA = const_cast<RemoteDbus_t*>(getRemoteDbus(SYS_PROFLIE_CONFIG_DATA->RemoteDbus_PROFILE));
    if (RemoteDbus_CONFIG_DATA) {
        motor[control::FourWheel::front_left] = fl_motor;
        motor[control::FourWheel::front_right] = fr_motor;
        motor[control::FourWheel::back_left] = bl_motor;
        motor[control::FourWheel::back_right] = br_motor;

        control::chassis_t chassis_data;
        chassis_data.motors = motor;
        chassis_data.model = RemoteDbus_CONFIG_DATA->Chassis_Model;
        chassis_data.omni_wheel_setup = RemoteDbus_CONFIG_DATA->OMNI_Wheel_setup_t;

        if (SuperCap != nullptr) {
            SuperCap();
            if (sys_state == SYSTEM_ERRORCONTROL) {
                return;
            }
            chassis_data.has_super_capacitor = true;
            chassis_data.super_capacitor = super_cap;
        } else {
            chassis_data.has_super_capacitor = false;
        }

        dbus = new remote::DBUS(RemoteDbus_CONFIG_DATA->huart);
        chassis = new control::Chassis(chassis_data);
        chassis->SetMaxMotorSpeed(RemoteDbus_CONFIG_DATA->Max_Speed);

    } else {
        resetindexCanBridge(SYS_PROFLIE_CONFIG_DATA->CanBridge_PROFILE);
        sys_state = SYSTEM_ERRORCONTROL;
    }
}

/* -- RemoteSbus --  */

void InitRemoteSbusChassis(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num],
                           void(*SuperCap(void))) {
    RemoteSbus_t* RemoteSbus_CONFIG_DATA = const_cast<RemoteSbus_t*>(getRemoteSbus(SYS_PROFLIE_CONFIG_DATA->RemoteSbus_PROFILE));
    if (RemoteSbus_CONFIG_DATA) {
        motor[control::FourWheel::front_left] = fl_motor;
        motor[control::FourWheel::front_right] = fr_motor;
        motor[control::FourWheel::back_left] = bl_motor;
        motor[control::FourWheel::back_right] = br_motor;

        control::chassis_t chassis_data;
        chassis_data.motors = motor;
        chassis_data.model = RemoteSbus_CONFIG_DATA->Chassis_Model;
        chassis_data.omni_wheel_setup = RemoteSbus_CONFIG_DATA->OMNI_Wheel_setup_t;

        if (SuperCap != nullptr) {
            SuperCap();
            if (sys_state == SYSTEM_ERRORCONTROL) {
                return;
            }
            chassis_data.has_super_capacitor = true;
            chassis_data.super_capacitor = super_cap;
        } else {
            chassis_data.has_super_capacitor = false;
        }

        sbus = new remote::SBUS(RemoteSbus_CONFIG_DATA->huart);
        chassis = new control::Chassis(chassis_data);
        chassis->SetMaxMotorSpeed(RemoteSbus_CONFIG_DATA->Max_Speed);

    } else {
        resetindexCanBridge(SYS_PROFLIE_CONFIG_DATA->CanBridge_PROFILE);
        sys_state = SYSTEM_ERRORCONTROL;
    }
}

/* -- Chassis Main --  */

// 在 chassis_task.cpp 中的 ChassisInit 函数中
void ChassisInit(driver::MotorCANBase* (&motor)[control::FourWheel::motor_num],
                 void(*SuperCap(void)), ChassisControlMode_t& ChassisControlMode, ChassisRemote_t& ChassisRemote) {
    ControlModel_t* ControlModel_CONFIG_DATA = const_cast<ControlModel_t*>(getControlModel(SYS_PROFLIE_CONFIG_DATA->ControlModel_PROFILE));
    if (ControlModel_CONFIG_DATA) {
        ChassisControlMode = ControlModel_CONFIG_DATA->ChassisControlMode;
        switch (ChassisControlMode) {
            case ModeDbus:
                if (const RemoteDbus_t* RemoteDbus = getRemoteDbus(SYS_PROFLIE_CONFIG_DATA->RemoteDbus_PROFILE)) {
                    ChassisRemote.remote_dbus = RemoteDbus;
                    InitRemoteDbusChassis(motor, SuperCap);
                } else {
                    resetindexRemoteDbus(SYS_PROFLIE_CONFIG_DATA->RemoteDbus_PROFILE);
                    sys_state = SYSTEM_ERRORCONTROL;
                }
                break;
            case ModeSbus:
                if (const RemoteSbus_t* RemoteSbus = getRemoteSbus(SYS_PROFLIE_CONFIG_DATA->RemoteSbus_PROFILE)) {
                    ChassisRemote.remote_sbus = RemoteSbus;
                    InitRemoteSbusChassis(motor, SuperCap);
                } else {
                    resetindexRemoteSbus(SYS_PROFLIE_CONFIG_DATA->RemoteSbus_PROFILE);
                    sys_state = SYSTEM_ERRORCONTROL;
                }
                break;
            case ModeCanBridge:
                // todo:在里面设置了CanBridge回调接收的ID不需要同步通道了
                InitCanBridgeChassis(motor, SuperCap);
                break;
            case ModeNone:
                break;
            default:
                break;
        }
    } else {
        resetindexControlModel(SYS_PROFLIE_CONFIG_DATA->ControlModel_PROFILE);
        sys_state = SYSTEM_ERRORCONTROL;
    }
}

void ChassisUpdate(ChassisControlMode_t& ChassisControlMode, ChassisRemote_t& ChassisRemote) {
    switch (ChassisControlMode) {
        case ModeDbus: {
            const RemoteDbus_t* remote_dbus = ChassisRemote.remote_dbus;
            chassis->SetSpeed(dbus->channels[remote_dbus->XSpeedChannel] * ratio,
                              dbus->channels[remote_dbus->YSpeedChannel] * ratio,
                              dbus->channels[remote_dbus->TurnSpeedChannel] * ratio);
            chassis->SetPower(false, 30, 20, 60);
            chassis->Update();
        } break;
        case ModeSbus: {
            const RemoteSbus_t* remote_sbus = ChassisRemote.remote_sbus;
            chassis->SetSpeed(sbus->channels[remote_sbus->XSpeedChannel] * ratio,
                              sbus->channels[remote_sbus->YSpeedChannel] * ratio,
                              sbus->channels[remote_sbus->TurnSpeedChannel] * ratio);
            chassis->SetPower(false, 30, 20, 60);
            chassis->Update();
        } break;
        case ModeCanBridge:
            chassis->Update();
            break;
        case ModeNone:
            break;
        default:
            // 处理未预期的枚举值
            break;
    }
}