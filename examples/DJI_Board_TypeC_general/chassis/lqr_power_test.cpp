/*###########################################################
 # Copyright (c) 2026. BNU-HKBU UIC RoboMaster              #
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

#include "main.h"

#include "bsp_batteryvol.h"
#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "dbus.h"
#include "power_limit_lqr.h"
#include "protocol.h"

bsp::CAN* can = nullptr;
driver::MotorCANBase* fl_motor = nullptr;
driver::MotorCANBase* fr_motor = nullptr;
driver::MotorCANBase* bl_motor = nullptr;
driver::MotorCANBase* br_motor = nullptr;

control::Chassis* chassis = nullptr;
remote::DBUS* dbus = nullptr;
bsp::BatteryVol* battery_vol = nullptr;
communication::Referee* referee = nullptr;

bsp::UART* REFUART = nullptr;

void RM_RTOS_Init() {
    osDelay(200);
    print_use_uart(&huart1, true, 921600);
    can = new bsp::CAN(&hcan1, false);
    fl_motor = new driver::Motor3508(can, 0x203);
    fr_motor = new driver::Motor3508(can, 0x201);
    bl_motor = new driver::Motor3508(can, 0x204);
    br_motor = new driver::Motor3508(can, 0x202);
    
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

    control::chassis_t chassis_data;
    chassis_data.motors = motors;
    chassis_data.model = control::CHASSIS_OMNI_WHEEL;
    chassis_data.power_limit_on = true;
    chassis_data.use_lqr_power_limit = true;
    chassis_data.season = control::SeasonRuleManager::SEASON_2026;
    chassis_data.blood_priority = true;
    chassis_data.omni_wheel_setup_.Center_point_angle = 45;
    chassis_data.omni_wheel_setup_.Wheel_angle_a = 45;
    chassis_data.omni_wheel_setup_.Wheel_angle_b = 45;
    chassis_data.omni_wheel_setup_.chassis_radius = 0.2f;
    chassis = new control::Chassis(chassis_data);

    dbus = new remote::DBUS(&BOARD_DBUS);
    battery_vol = new bsp::BatteryVol(&hadc3, ADC_CHANNEL_8, 1, ADC_SAMPLETIME_3CYCLES);
    REFUART = new bsp::UART(&huart6);
    REFUART->SetBaudrate(115200);
    REFUART->SetupRx(300);
    REFUART->SetupTx(300);
    referee = new communication::Referee(REFUART);
    osDelay(100);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    osDelay(500);

    while (true) {
        clear_screen();
        set_cursor(0, 0);
        const float ratio = 1.0f / 660.0f * 6 * PI;
        chassis->SetSpeed(dbus->ch0 * ratio, dbus->ch1 * ratio, dbus->ch2 * ratio);

        if (dbus->swl == remote::UP || dbus->swl == remote::DOWN) {
            RM_ASSERT_TRUE(false, "Operation killed");
        }
        
        float max_power = 75.0f;
        uint8_t buffer_percent = 50;
        
        if (referee->IsOnline()) {
            max_power = referee->power_heat_data.chassis_power;
            buffer_percent = static_cast<uint8_t>(
                referee->power_heat_data.chassis_power_buffer * 60.0f / 100.0f
            );
        }
        
        chassis->UpdatePower(true, max_power, battery_vol->GetBatteryVol(), buffer_percent);
        chassis->Update();
        
        static uint32_t print_tick = 0;
        if (HAL_GetTick() - print_tick > 100) {
            print_tick = HAL_GetTick();
            print("P_est: %.2fW, P_pred: %.2fW, Buffer: %.1fJ, Warn: %d, Scale: %.2f\r\n",
                  chassis->GetPowerEstimate(),
                  chassis->GetPowerPrediction(),
                  chassis->GetBuffer(),
                  chassis->GetWarnType(),
                  chassis->GetSpeedScale());
        }
        
        osDelay(10);
    }
}
