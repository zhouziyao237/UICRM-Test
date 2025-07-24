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

#include "motor_error.h"

namespace error {

    // 新状态机实现
    void init_motor_state(MotorStateInfo* state) {
        state->state = MOTOR_STATE_STOPPED;
        state->filtered_accel = 0.0f;
        state->position_lock_time = 0.0f;
        state->state_timer = 0.0f;
        state->cmd_info.cmd_speed = 0.0f;
        state->cmd_info.last_cmd_speed = 0.0f;
        state->cmd_info.cmd_change_timer = 0.0f;
        state->last_valid_speed = 0.0f;
    }

    MotorState update_motor_state(MotorStateInfo* state, const MotorStateConfig* config,
                                  float current_speed, float current_position, float cmd_speed) {
        static float last_position = 0.0f;
        static float last_speed = 0.0f;
        const float Ts = 0.002f;  // 2ms采样周期

        // 更新指令速度信息
        state->cmd_info.last_cmd_speed = state->cmd_info.cmd_speed;
        state->cmd_info.cmd_speed = cmd_speed;

        // 检测指令变化 (rad/s)
        float cmd_change = fabsf(state->cmd_info.cmd_speed - state->cmd_info.last_cmd_speed);
        if (cmd_change > config->cmd_change_thres) {
            state->cmd_info.cmd_change_timer = 0.0f;  // 重置变化计时器
        } else {
            state->cmd_info.cmd_change_timer += Ts;  // 增加稳定时间
        }

        // 1. 计算实时加速度 (rad/s²)
        float accel = (current_speed - last_speed) / Ts;
        state->filtered_accel = 0.3f * accel + 0.7f * state->filtered_accel;

        // 2. 位置变化检测 (rad)
        float position_delta = fabsf(current_position - last_position);
        if (position_delta < config->position_lock_thres) {
            state->position_lock_time += Ts;
        } else {
            state->position_lock_time = 0.0f;
            state->last_valid_speed = current_speed;  // 记录有效速度 (rad/s)
        }

        // 3. 状态转移逻辑
        switch (state->state) {
            case MOTOR_STATE_STOPPED:
                // 从停止到启动的转移条件
                if (fabsf(cmd_speed) > config->cmd_speed_thres) {
                    state->state = MOTOR_STATE_STARTING;
                    state->state_timer = 0.0f;  // 重置计时器
                }
                break;

            case MOTOR_STATE_STARTING:
                state->state_timer += Ts;

                // 启动成功条件：达到目标速度比例或指令稳定
                if (fabsf(current_speed) > fabsf(cmd_speed) * config->start_speed_ratio ||
                    state->cmd_info.cmd_change_timer > 0.1f) {  // 指令稳定100ms
                    state->state = MOTOR_STATE_RUNNING;
                }
                // 启动失败：超时仍未达到速度
                else if (state->state_timer > config->start_time_thres) {
                    state->state = MOTOR_STATE_STALLED;
                }
                break;

            case MOTOR_STATE_RUNNING: {
                // 动态阈值调整 - 使用当前指令速度 (rad/s)
                float dyn_accel_thres =
                    config->accel_thres * (1.0f + config->dynamic_factor * fabsf(cmd_speed));
                float dyn_speed_thres = (fabsf(cmd_speed) > config->cmd_speed_thres)
                                            ? config->zero_speed_thres
                                            : (2.0f * config->zero_speed_thres);

                // 检查堵转条件
                bool sudden_stall = (state->filtered_accel < -dyn_accel_thres) &&
                                    (fabsf(current_speed) < dyn_speed_thres);

                bool position_stall = state->position_lock_time > config->lock_time_thres;

                // 速度异常下降检测 (rad/s)
                bool speed_drop = (fabsf(current_speed) < fabsf(state->last_valid_speed) * 0.5f) &&
                                  (state->cmd_info.cmd_change_timer > 0.05f);  // 指令稳定50ms

                if (sudden_stall || position_stall || speed_drop) {
                    state->state = MOTOR_STATE_STALLED;
                }
                // 检查是否停止
                else if (fabsf(cmd_speed) < config->cmd_speed_thres) {
                    state->state = MOTOR_STATE_STOPPED;
                }
                // 检查目标值变化是否需要重新启动
                else if (cmd_change > config->cmd_change_thres) {
                    state->state = MOTOR_STATE_STARTING;
                    state->state_timer = 0.0f;
                }
                break;
            }

            case MOTOR_STATE_STALLED:
                // 从堵转恢复的条件：收到零指令或位置解锁
                if (fabsf(cmd_speed) < config->cmd_speed_thres ||
                    position_delta > config->position_lock_thres) {
                    state->state = MOTOR_STATE_STOPPED;
                }
                break;

            default:
                // 处理未知状态
                state->state = MOTOR_STATE_STOPPED;
                break;
        }

        // 更新历史值
        last_speed = current_speed;
        last_position = current_position;

        return state->state;
    }

    /**
     * @brief  统计在目标值附近±百分比范围内的数据点数量
     * @param  data: 数据数组指针
     * @param  size: 数据数组大小
     * @param  target: 目标值
     * @param  tolerance_percent: 容差百分比 (0.0-1.0)
     * @retval 在范围内的数据点数量
     */
    uint8_t count_near(float* data, uint8_t size, float target, float tolerance_percent) {
        // 计算允许的波动范围
        float tolerance = fabsf(target) * tolerance_percent;  // 使用浮点绝对值函数
        float lower_bound = target - tolerance;
        float upper_bound = target + tolerance;

        uint8_t count = 0;

        // 遍历所有数据点
        for (uint8_t i = 0; i < size; i++) {
            // 检查是否在范围内
            if (data[i] >= lower_bound && data[i] <= upper_bound) {
                count++;
            }
        }

        return count;
    }

    DropDetectionResult detect_all_drops(float* data, uint8_t size, DropDetectionConfig config) {
        DropDetectionResult result = {0, 0, 0, 0};

        // 1. 相邻点比较检测
        for (uint8_t i = 1; i < size; i++) {
            float change = ((data[i - 1] - data[i]) / data[i - 1]) * 100.0f;
            if (change > config.drop_threshold_percent) {
                result.adjacent_drops++;
            }
        }

        // 2. 滑动窗口检测
        for (uint8_t i = config.window_size; i < size; i++) {
            float window_avg = 0;
            for (uint8_t j = i - config.window_size; j < i; j++) {
                window_avg += data[j];
            }
            window_avg /= config.window_size;

            float diff = ((window_avg - data[i]) / window_avg) * 100.0f;
            if (diff > config.drop_threshold_percent) {
                result.window_drops++;
            }
        }

        // 3. 导数检测
        float derivatives[size - 1];
        float mean = 0, std_dev = 0;

        // 计算导数
        for (uint8_t i = 0; i < size - 1; i++) {
            derivatives[i] = data[i + 1] - data[i];
            mean += derivatives[i];
        }
        mean /= (size - 1);

        // 计算标准差
        for (uint8_t i = 0; i < size - 1; i++) {
            std_dev += (derivatives[i] - mean) * (derivatives[i] - mean);
        }
        std_dev = sqrtf(std_dev / (size - 2));

        // 检测异常负导数
        for (uint8_t i = 0; i < size - 1; i++) {
            if (derivatives[i] < 0 &&
                (mean - derivatives[i]) > config.std_dev_multiplier * std_dev) {
                result.derivative_drops++;
            }
        }

        // 综合结果
        result.total_drops = result.adjacent_drops + result.window_drops + result.derivative_drops;

        return result;
    }

}  // namespace error