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
#include "bsp_can.h"
#include "chassis.h"
#include "config_eeprom/bsp_pg.h"

enum ChassisControlMode_t {
    ModeNone = 0,
    ModeCanBridge = 1,
    ModeDbus = 2,
    ModeSbus = 3,
};

typedef struct {
    ChassisControlMode_t ChassisControlMode = ModeDbus;

} __attribute__((packed)) ControlModel_t;

PG_DECLARE_ARRAY(ControlModel_t, MAX_PROFILE_COUNT, ControlModel);

typedef struct {
    uint8_t ControlModel_PROFILE = ControlModel_PROFILE_COUNT;
    uint8_t MotorPID_PROFILE = MotorPID_PROFILE_COUNT;
    uint8_t MotorId_PROFILE = MotorId_PROFILE_COUNT;
    uint8_t CanBridge_PROFILE = CanBridge_PROFILE_COUNT;
    uint8_t RemoteDbus_PROFILE = RemoteDbus_PROFILE_COUNT;
    uint8_t RemoteSbus_PROFILE = RemoteSbus_PROFILE_COUNT;
    uint8_t SuperCap_PROFILE = SuperCap_PROFILE_COUNT;
} __attribute__((packed)) PROFILE_CONFIG_t;

PG_DECLARE_ARRAY(PROFILE_CONFIG_t, PROFILE_CONFIG_MAX_PROFILE_COUNT, PROFILECONFIG);

typedef struct {
    float kp = 2500.0f;  /// 比例系数
    float ki = 3.0f;     /// 积分系数
    float kd = 0.0f;     /// 微分系数

    float max_out = 30000;   /// 输出限幅
    float max_iout = 10000;  /// 积分输出限幅
    float deadband = 0;      /// 死区
    /// 变速积分
    float A = 3 * PI;                                              /// 变速积分所能达到的最大值为A+B
    float B = 2 * PI;                                              /// 启动变速积分的死区
    float output_filtering_coefficient = 0.1;                      /// 输出滤波系数
    float derivative_filtering_coefficient = 0;                    /// 微分滤波系数
    uint8_t mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                   control::ConstrainedPID::OutputFilter |         // 输出滤波
                   control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                   control::ConstrainedPID::ChangingIntegralRate;  // 变速积分
} __attribute__((packed)) MotorPID_t;

PG_DECLARE_ARRAY(MotorPID_t, MAX_PROFILE_COUNT, MotorPid);

typedef struct {
    CAN_HandleTypeDef* hcan = &hcan2;
    bool is_master = false;
    uint16_t fl_Id = 0x202;
    uint16_t fr_Id = 0x201;
    uint16_t bl_Id = 0x203;
    uint16_t br_Id = 0x204;
} __attribute__((packed)) MotorId_t;

PG_DECLARE_ARRAY(MotorId_t, MAX_PROFILE_COUNT, MotorId);

typedef struct {
    CAN_HandleTypeDef* hcan = &hcan1;
    bool is_master = true;
    uint16_t CanBridge_RxId = 0x52;
    uint16_t CanBridge_TxId = 0x51;
    uint16_t CanBridge_XYWrapper = 0x70;
    uint16_t CanBridge_TurnWrapper = 0x71;
    uint16_t CanBridge_PowerLimitWrapper = 0x72;
    uint16_t CanBridge_CurrentPowerWrapper = 0x73;
    float Max_Speed = 2 * PI * 7;
    control::chassis_model_t Chassis_Model = control::CHASSIS_MECANUM_WHEEL;
    control::OMNI_Wheel_setup_t OMNI_Wheel_setup_t = {45, 45, 45, 0.15f};
} __attribute__((packed)) CanBridge_t;

PG_DECLARE_ARRAY(CanBridge_t, MAX_PROFILE_COUNT, CanBridge);

typedef struct {
#if defined(DJI_Board_TypeA_general)
    UART_HandleTypeDef* huart = &huart1;
#elif defined(DJI_Board_TypeC_general)
    UART_HandleTypeDef* huart = &huart3;
#else
#error "Please select a board type"
#endif
    bool Enable_keyboard = false;
    int16_t XSpeedChannel = 1;
    int16_t YSpeedChannel = 0;
    int16_t TurnSpeedChannel = 2;
    float Max_Speed = 2 * PI * 7;
    control::chassis_model_t Chassis_Model = control::CHASSIS_MECANUM_WHEEL;
    control::OMNI_Wheel_setup_t OMNI_Wheel_setup_t = {45, 45, 45, 0.15f};
} __attribute__((packed)) RemoteDbus_t;

PG_DECLARE_ARRAY(RemoteDbus_t, MAX_PROFILE_COUNT, RemoteDbus);

typedef struct {
#if defined(DJI_Board_TypeA_general)
    UART_HandleTypeDef* huart = &huart1;
#elif defined(DJI_Board_TypeC_general)
    UART_HandleTypeDef* huart = &huart3;
#else
#error "Please select a board type"
#endif
    bool Enable_keyboard = false;
    int16_t XSpeedChannel = 1;
    int16_t YSpeedChannel = 0;
    int16_t TurnSpeedChannel = 2;
    float Max_Speed = 2 * PI * 7;
    control::chassis_model_t Chassis_Model = control::CHASSIS_MECANUM_WHEEL;
    control::OMNI_Wheel_setup_t OMNI_Wheel_setup_t = {45, 45, 45, 0.15f};
} __attribute__((packed)) RemoteSbus_t;

PG_DECLARE_ARRAY(RemoteSbus_t, MAX_PROFILE_COUNT, RemoteSbus);

typedef struct {
    CAN_HandleTypeDef* hcan = &hcan2;
    bool Enable = true;
    uint16_t tx_id = 0x02E;
    uint16_t tx_settings_id = 0x02F;
    uint16_t rx_id = 0x030;
    float MaxVoltage = 24.0f;
    float PowerTotal = 100.0f;
    float MaxChargePower = 150.0f;
    float MaxDischargePower = 250.0f;
    float PerferBuffer = 50.0f;
} __attribute__((packed)) SuperCap_t;

PG_DECLARE_ARRAY(SuperCap_t, MAX_PROFILE_COUNT, SuperCap);

typedef union {
    // todo:在里面设置了CanBridge回调接收的ID不需要同步通道了
    const RemoteDbus_t* remote_dbus;
    const RemoteSbus_t* remote_sbus;
} ChassisRemote_t;
