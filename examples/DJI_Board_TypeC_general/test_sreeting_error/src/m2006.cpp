// Copyright (c) 2023-2025. BNU-HKBU UIC RoboMaster
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

#include "MotorCanBase.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor_error.h"
#include "pid.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

// Refer to typeA datasheet for channel detail
static bsp::CAN* can1 = nullptr;
static driver::Motor2006* steering_motor = nullptr;

void Motor_Error(driver::Motor2006* Motor, const error::MotorStateConfig& motor_config,
                 error::MotorStateInfo& motor_state, float current_target, void (*FuncPtr)(void)) {
    // 实时状态检测 (每2ms检测一次)
    float current_speed = Motor->GetOutputShaftOmega();  // rad/s
    float current_angle = Motor->GetOutputShaftTheta();  // rad

    error::MotorState state =
        error::update_motor_state(&motor_state,
            &motor_config,
            current_speed,
            current_angle,
            current_target  // 使用当前目标值作为指令速度 (rad/s)
        );

    // 状态处理
    switch (state) {
        case error::MOTOR_STATE_STOPPED:
            // 停止状态
            break;

        case error::MOTOR_STATE_STARTING:
            // 启动状态
            print("Motor STARTING... Target: %.2f rad/s, Time: %.1f/%.1f ms\r\n", current_target,
                  motor_state.state_timer * 1000, motor_config.start_time_thres * 1000);
            break;

        case error::MOTOR_STATE_RUNNING:
            // 正常运行
            break;

        case error::MOTOR_STATE_STALLED:
            // 堵转保护
            Motor->SetTarget(0);  // 立即停止电机
            print("!!! MOTOR STALL DETECTED !!! Target: %.2f rad/s\r\n", current_target);
            // 等待50ms后重置
            osDelay(50);
            error::init_motor_state(&motor_state);
            current_target = 0.0f;  // 重置目标值
            FuncPtr();
            break;

        default:
            // 未知状态处理
            break;
    }

    // 状态显示
    const char* state_names[] = {"STOPPED", "STARTING", "RUNNING", "STALLED"};

    Motor->PrintData();
    print("Target: %.2f rad/s | State: %s\r\n", current_target, state_names[state]);
    print("Speed: %.2f rad/s | Accel: %.1f rad/s²\r\n", current_speed, motor_state.filtered_accel);
    print("PosLock: %.1f/%.1f ms | CmdChg: %.1f ms\r\n", motor_state.position_lock_time * 1000,
          motor_config.lock_time_thres * 1000, motor_state.cmd_info.cmd_change_timer * 1000);

    osDelay(2);  // 2ms周期
}

void RM_RTOS_Init() {
    print_use_uart(&huart8);
    can1 = new bsp::CAN(&hcan1, true);
    steering_motor = new driver::Motor2006(can1, 0x201);
    steering_motor->SetTransmissionRatio(36);

    control::ConstrainedPID::PID_Init_t steering_motor_omega_pid_init = {
        .kp = 50,
        .ki = 0,
        .kd = 1000,
        .max_out = 10000,
        .max_iout = 4000,
        .deadband = 0,                                          // 死区
        .A = 3 * M_PI,                                          // 变速积分所能达到的最大值为A+B
        .B = 2 * M_PI,                                          // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                    // 输出滤波系数
        .derivative_filtering_coefficient = 0,                  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate   // 变速积分
    };

    steering_motor->ReInitPID(steering_motor_omega_pid_init,
                              driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    steering_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL |
                            driver::MotorCANBase::ABSOLUTE);

    // Snail need to be run at idle throttle for some
    HAL_Delay(1000);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);

    // 初始化电机状态检测 (使用弧度值)
    error::MotorStateConfig motor_config = {
        .zero_speed_thres = 0.5f,      // 0.5 rad/s ≈ 5 RPM
        .accel_thres = 300.0f,         // 300 rad/s²
        .position_lock_thres = 0.01f,  // 位置变化阈值 (0.01 rad ≈ 0.57°)
        .lock_time_thres = 0.002f,     // 200 ms锁定时间
        .cmd_speed_thres = 2.0f,       // 2 rad/s指令速度阈值
        .dynamic_factor = 0.5f,        // 动态调整系数
        .start_time_thres = 0.5f,      // 启动超时0.5秒
        .start_speed_ratio = 0.7f,     // 达到70%目标速度视为启动成功
        .cmd_change_thres = 1.0f       // 指令变化阈值1.0 rad/s
    };

    error::MotorStateInfo steering_motor_state;
    error::init_motor_state(&steering_motor_state);

    // 目标值变量 (使用弧度值)
    float current_target = 0.0f;  // rad/s

    while (true) {
        set_cursor(0, 0);
        clear_screen();
        if (key.Read() == 0) {
            osDelay(30);
            if (key.Read() == 1)
                continue;
            while (key.Read() == 0) {
                osDelay(30);
            }

            // 切换目标值 (使用弧度值)
            if (current_target < 0.1f) {
                current_target = 10 * PI;  // 2 * 5π rad/s ≈ 60 * 5 RPM
            } else {
                current_target = 0.0f;  // 停止
            }
            steering_motor->SetTarget(current_target);
        }
        Motor_Error(steering_motor, motor_config, steering_motor_state, current_target, nullptr);
    }
}