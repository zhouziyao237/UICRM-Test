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

    // ---------------------------------- 摩擦轮 ----------------------------------

    // 添加双电机状态信息结构
    typedef struct {
        float speed;              // 当前速度 (rad/s)
        float position;           // 当前位置 (rad)
        float timestamp;          // 时间戳 (s)
    } MotorSample;

    typedef struct {
        MotorSample motor1_samples[50];   // 电机1采样数据
        MotorSample motor2_samples[50];   // 电机2采样数据
        uint8_t sample_count;             // 当前采样数量
        uint8_t max_samples;              // 最大采样数量
        float sample_period;              // 采样周期 (s)
        float decel_threshold;            // 减速度阈值 (rad/s²)
        float speed_drop_ratio;           // 速度下降比例阈值 (0-1)
        uint32_t last_check_time_ms;      // 上次检查时间 (ms)
        bool motor1_decel_detected;       // 电机1减速检测标志
        bool motor2_decel_detected;       // 电机2减速检测标志
        bool both_decel_detected;         // 双电机同时减速检测标志
    } DualMotorDecelDetector;

    // 添加结果结构
    typedef struct {
        bool motor1_rapid_decel;          // 电机1是否骤减
        bool motor2_rapid_decel;          // 电机2是否骤减
        bool dual_motor_sync_decel;       // 双电机同步骤减
        float motor1_current_accel;       // 电机1当前加速度
        float motor2_current_accel;       // 电机2当前加速度
        float time_since_last_sample;     // 自上次采样的时间
    } DualMotorDecelResult;

    // 初始化双电机骤减检测器
    void init_dual_motor_decel_detector(DualMotorDecelDetector* detector,
                                       float sample_period,
                                       float decel_threshold,
                                       float speed_drop_ratio);

    // 非阻塞采样更新函数
    bool update_dual_motor_decel_detection(DualMotorDecelDetector* detector,
                                          float motor1_speed,
                                          float motor1_position,
                                          float motor2_speed,
                                          float motor2_position,
                                          uint32_t current_time_ms);

    // 获取检测结果
    DualMotorDecelResult get_dual_motor_decel_result(DualMotorDecelDetector* detector);

}

#endif //MOTOR_ERROR_H