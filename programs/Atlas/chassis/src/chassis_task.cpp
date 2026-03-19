/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "chassis_task.h"

#include "bsp_os.h"
#include "bsp_print.h"
#include "chassis.h"
#include "user_define.h"

driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;
control::Chassis* chassis = nullptr;
bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

driver::SuperCap* super_cap = nullptr;
communication::CanBridge* can_bridge = nullptr;

void init_chassis() {
    // 初始化CAN
    can1 = new bsp::CAN(&hcan1, true, 8);

    can2 = new bsp::CAN(&hcan2, true, 8);

    // 初始化电机
    fl_motor = new driver::Motor3508(can1, 0x204);
    fr_motor = new driver::Motor3508(can1, 0x203);
    bl_motor = new driver::Motor3508(can1, 0x202);
    br_motor = new driver::Motor3508(can1, 0x201);
    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 2000,
        .ki = 0,
        .kd = 0,
        .max_out = 30000,
        .max_iout = 0,
        .deadband = 0,
        .A = 3 * PI,
        .B = 2 * PI,
        .output_filtering_coefficient = 0.05,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::Integral_Limit |
                control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate,
    };

    fl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    fl_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL);
    fl_motor->SetTransmissionRatio(19);

    fr_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    fr_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL);
    fr_motor->SetTransmissionRatio(19);

    bl_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    bl_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL);
    bl_motor->SetTransmissionRatio(19);

    br_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    br_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL);
    br_motor->SetTransmissionRatio(19);

    driver::MotorCANBase* motors[control::FourWheel::motor_num];
    motors[control::FourWheel::front_left] = fl_motor;
    motors[control::FourWheel::front_right] = fr_motor;
    motors[control::FourWheel::back_left] = bl_motor;
    motors[control::FourWheel::back_right] = br_motor;

    can_bridge = new communication::CanBridge(can2, 0x52);

    // 底盘初始化
    control::chassis_t chassis_data;

    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_OMNI_WHEEL;
    chassis_data.power_limit_on = true;
    chassis_data.use_lqr_power_limit = true;
    chassis_data.season = control::SeasonRuleManager::SEASON_2026;
    chassis_data.blood_priority = true;

    chassis_data.model = control::CHASSIS_OMNI_WHEEL;
    chassis_data.omni_wheel_setup_.Center_point_angle = 45;
    chassis_data.omni_wheel_setup_.Wheel_angle_a = 45;
    chassis_data.omni_wheel_setup_.Wheel_angle_b = 45;
    chassis_data.omni_wheel_setup_.chassis_radius = 0.2f;
    chassis = new control::Chassis(chassis_data);

    // chassis->SetMaxMotorSpeed(2 * 7 * PI);

    chassis->CanBridgeSetTxId(0x51);
    can_bridge->RegisterRxCallback(0x69, chassis->CanBridgeUpdateEventXYWrapper, chassis);
    can_bridge->RegisterRxCallback(0x71, chassis->CanBridgeUpdateEventTurnWrapper, chassis);
    can_bridge->RegisterRxCallback(0x72, chassis->CanBridgeUpdateEventPowerLimitWrapper, chassis);
    can_bridge->RegisterRxCallback(0x73, chassis->CanBridgeUpdateEventCurrentPowerWrapper, chassis);

    HAL_Delay(300);
}

void chassisMain() {
    // chassis->UpdatePower(true, 50, 22, 60);
    chassis->Update();
}
