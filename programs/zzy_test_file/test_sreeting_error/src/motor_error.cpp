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

#include "../../../../boards/platform/stm32h7/include/bsp_os.h"

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

    // ---------------------------------- 摩擦轮 ----------------------------------

    void init_dual_motor_decel_detector(DualMotorDecelDetector* detector,
                                   float sample_period,
                                   float decel_threshold,
                                   float speed_drop_ratio) {
        detector->sample_period = sample_period;
        detector->decel_threshold = decel_threshold;
        detector->speed_drop_ratio = speed_drop_ratio;
        detector->sample_count = 0;
        detector->max_samples = 50;  // 可根据需要调整
        detector->last_check_time_ms = 0;
        detector->motor1_decel_detected = false;
        detector->motor2_decel_detected = false;
        detector->both_decel_detected = false;

        // 清空采样数组
        for(int i = 0; i < detector->max_samples; i++) {
            detector->motor1_samples[i].speed = 0.0f;
            detector->motor1_samples[i].position = 0.0f;
            detector->motor1_samples[i].timestamp = 0.0f;
            detector->motor2_samples[i].speed = 0.0f;
            detector->motor2_samples[i].position = 0.0f;
            detector->motor2_samples[i].timestamp = 0.0f;
        }
    }

    bool update_dual_motor_decel_detection(DualMotorDecelDetector* detector,
                                          float motor1_speed,
                                          float motor1_position,
                                          float motor2_speed,
                                          float motor2_position,
                                          uint32_t current_time_ms) {
        // 非阻塞检查：只有在足够的时间间隔后才执行检测
        if((current_time_ms - detector->last_check_time_ms) < (uint32_t)(detector->sample_period * 1000)) {
            return false;  // 不满足采样时间要求，返回false
        }

        // 更新采样数据
        uint8_t next_index = detector->sample_count % detector->max_samples;

        detector->motor1_samples[next_index].speed = motor1_speed;
        detector->motor1_samples[next_index].position = motor1_position;
        detector->motor2_samples[next_index].speed = motor2_speed;
        detector->motor2_samples[next_index].position = motor2_position;

        // 更新时间戳 (转换为秒)
        detector->motor1_samples[next_index].timestamp = current_time_ms / 1000.0f;
        detector->motor2_samples[next_index].timestamp = current_time_ms / 1000.0f;

        detector->sample_count++;
        detector->last_check_time_ms = current_time_ms;

        // 如果采样数据不足，无法计算加速度
        if(detector->sample_count < 2) {
            return false;
        }

        // 计算加速度
        uint8_t prev_index = (next_index == 0) ? detector->max_samples - 1 : next_index - 1;

        float dt1 = detector->motor1_samples[next_index].timestamp - detector->motor1_samples[prev_index].timestamp;
        float dt2 = detector->motor2_samples[next_index].timestamp - detector->motor2_samples[prev_index].timestamp;

        // 防止除零错误
        if(dt1 <= 0.0f || dt2 <= 0.0f) {
            return false;
        }

        float accel1 = (detector->motor1_samples[next_index].speed - detector->motor1_samples[prev_index].speed) / dt1;
        float accel2 = (detector->motor2_samples[next_index].speed - detector->motor2_samples[prev_index].speed) / dt2;

        // 检测骤减：加速度小于负阈值（即快速减速）
        bool motor1_rapid_decel = (accel1 < -detector->decel_threshold);
        bool motor2_rapid_decel = (accel2 < -detector->decel_threshold);

        // 检测速度骤降比例
        if(fabsf(detector->motor1_samples[prev_index].speed) > 0.001f) {
            float speed_drop_ratio1 = fabsf((detector->motor1_samples[prev_index].speed -
                                           detector->motor1_samples[next_index].speed) /
                                           detector->motor1_samples[prev_index].speed);
            if(speed_drop_ratio1 > detector->speed_drop_ratio) {
                motor1_rapid_decel = true;
            }
        }

        if(fabsf(detector->motor2_samples[prev_index].speed) > 0.001f) {
            float speed_drop_ratio2 = fabsf((detector->motor2_samples[prev_index].speed -
                                           detector->motor2_samples[next_index].speed) /
                                           detector->motor2_samples[prev_index].speed);
            if(speed_drop_ratio2 > detector->speed_drop_ratio) {
                motor2_rapid_decel = true;
            }
        }

        // 更新检测标志
        detector->motor1_decel_detected = motor1_rapid_decel;
        detector->motor2_decel_detected = motor2_rapid_decel;
        detector->both_decel_detected = motor1_rapid_decel && motor2_rapid_decel;

        return true;  // 成功完成一次检测
    }

    DualMotorDecelResult get_dual_motor_decel_result(DualMotorDecelDetector* detector) {
        DualMotorDecelResult result = {0};

        // 获取最新采样数据计算当前加速度
        if(detector->sample_count >= 2) {
            uint8_t current_idx = (detector->sample_count - 1) % detector->max_samples;
            uint8_t prev_idx = (current_idx == 0) ? detector->max_samples - 1 : current_idx - 1;

            float dt1 = detector->motor1_samples[current_idx].timestamp - detector->motor1_samples[prev_idx].timestamp;
            float dt2 = detector->motor2_samples[current_idx].timestamp - detector->motor2_samples[prev_idx].timestamp;

            if(dt1 > 0.0f) {
                result.motor1_current_accel = (detector->motor1_samples[current_idx].speed -
                                             detector->motor1_samples[prev_idx].speed) / dt1;
            }

            if(dt2 > 0.0f) {
                result.motor2_current_accel = (detector->motor2_samples[current_idx].speed -
                                             detector->motor2_samples[prev_idx].speed) / dt2;
            }
        }

        result.motor1_rapid_decel = detector->motor1_decel_detected;
        result.motor2_rapid_decel = detector->motor2_decel_detected;
        result.dual_motor_sync_decel = detector->both_decel_detected;
        result.time_since_last_sample = (float)(bsp::GetHighresTickMilliSec() - detector->last_check_time_ms) / 1000.0f;  // 需要确保millis可用

        return result;
    }


}  // namespace error