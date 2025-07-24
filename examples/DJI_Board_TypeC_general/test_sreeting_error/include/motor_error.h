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
// Created by Administrator on 2025/7/19.
//

#ifndef MOTOR_ERROR_H
#define MOTOR_ERROR_H

#include "stdint.h"
#include <math.h>

namespace error {
    typedef enum {
        MOTOR_STATE_STOPPED,
        MOTOR_STATE_STARTING,
        MOTOR_STATE_RUNNING,
        MOTOR_STATE_STALLED
    } MotorState;

    // 指令速度信息结构
    typedef struct {
        float cmd_speed;           // 当前指令速度 (rad/s)
        float last_cmd_speed;      // 上一次指令速度 (rad/s)
        float cmd_change_timer;    // 指令变化计时器 (s)
    } CmdSpeedInfo;

    typedef struct {
        float zero_speed_thres;       // 零速阈值 (rad/s)
        float accel_thres;            // 加速度阈值 (rad/s²)
        float position_lock_thres;    // 位置锁定阈值 (rad)
        float lock_time_thres;        // 锁定时间阈值 (s)
        float cmd_speed_thres;        // 指令速度阈值 (rad/s)
        float dynamic_factor;         // 动态调整系数
        float start_time_thres;       // 启动超时阈值 (s)
        float start_speed_ratio;      // 启动成功速度比例 (0-1)
        float cmd_change_thres;       // 指令变化阈值 (rad/s)
    } MotorStateConfig;

    typedef struct {
        MotorState state;             // 当前状态
        float filtered_accel;         // 滤波后加速度 (rad/s²)
        float position_lock_time;     // 位置锁定计时 (s)
        float state_timer;            // 状态持续时间 (s)
        CmdSpeedInfo cmd_info;        // 指令速度信息
        float last_valid_speed;       // 上次有效速度 (rad/s)
    } MotorStateInfo;

    void init_motor_state(MotorStateInfo* state);
    MotorState update_motor_state(MotorStateInfo* state,
                                 const MotorStateConfig* config,
                                 float current_speed,
                                 float current_position,
                                 float cmd_speed);

    // 原始函数保持不动
    typedef struct {
        float drop_threshold_percent;  // 相邻点下降阈值
        uint8_t window_size;           // 滑动窗口大小
        float std_dev_multiplier;      // 标准差倍数阈值
    } DropDetectionConfig;

    typedef struct {
        uint8_t adjacent_drops;    // 基于相邻点检测到的骤降
        uint8_t window_drops;      // 基于窗口检测到的骤降
        uint8_t derivative_drops;  // 基于导数检测到的骤降
        uint8_t total_drops;       // 总骤降次数
    } DropDetectionResult;

    uint8_t count_near(float *data, uint8_t size, float target, float tolerance_percent);
    DropDetectionResult detect_all_drops(float* data, uint8_t size, DropDetectionConfig config);
}

#endif //MOTOR_ERROR_H