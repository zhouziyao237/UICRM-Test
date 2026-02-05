// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
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
// Created by Administrator on 2026/1/16.
//

#include "StallDetector.h"
#include "utils.h"
#include "bsp_os.h"
#include <cmath>

namespace driver {

    StallDetector::StallDetector()
        : filtered_speed_(0),
          filtered_current_(0),
          is_stalled_(false),
          stall_start_time_(0),
          last_update_time_(0) {}

    void StallDetector::Init(const StallConfig& config) {
        config_ = config;

        // 初始化卡尔曼滤波器
        speed_filter_.Init(config.speed_kf_config);
        current_filter_.Init(config.current_kf_config);

        // 重置状态
        Reset();
    }

    void StallDetector::Update(float actual_current, float target_current, float actual_speed,
                               uint32_t timestamp) {
        // 更新时间间隔
        last_update_time_ = timestamp;

        // 应用卡尔曼滤波
        filtered_speed_ = speed_filter_.Update(actual_speed);
        filtered_current_ = current_filter_.Update(actual_current);

        // 检查堵转条件
        bool stall_condition = CheckStallCondition(filtered_current_, target_current, filtered_speed_);

        if (stall_condition) {
            // 首次检测到堵转
            if (!is_stalled_) {
                stall_start_time_ = timestamp;
                is_stalled_ = true;
            }
        } else {
            // 清除堵转状态
            if (is_stalled_) {
                is_stalled_ = false;
                stall_start_time_ = 0;
            }
        }
    }

    bool StallDetector::CheckStallCondition(float filtered_current, float target_current,
                                           float filtered_speed) {
        // 检查目标电流是否足够大（避免低电流时误报）
        if (fabsf(target_current) < config_.min_target_current) {
            return false;
        }

        // 计算实际电流与目标电流的比例
        float current_ratio = fabsf(filtered_current) / fabsf(target_current);

        // 检查速度是否过低
        bool speed_low = fabsf(filtered_speed) < config_.speed_threshold;

        // 检查电流比例是否过高
        bool current_high = current_ratio > config_.current_ratio_threshold;

        // 堵转条件：电流比例高且速度低
        return speed_low && current_high;
    }

    bool StallDetector::IsStalled() const {
        if (!is_stalled_) {
            return false;
        }

        // 检查堵转持续时间是否超过检测时间窗口
        uint32_t current_time = bsp::GetHighresTickMilliSec();
        uint32_t stall_duration = current_time - stall_start_time_;

        return stall_duration >= config_.detection_time_ms;
    }

    uint32_t StallDetector::GetStallDuration() const {
        if (!is_stalled_) {
            return 0;
        }

        uint32_t current_time = bsp::GetHighresTickMilliSec();
        return current_time - stall_start_time_;
    }

    void StallDetector::Reset() {
        is_stalled_ = false;
        stall_start_time_ = 0;
        last_update_time_ = 0;

        speed_filter_.Reset();
        current_filter_.Reset();
    }

    float StallDetector::GetFilteredSpeed() const {
        return filtered_speed_;
    }

    float StallDetector::GetFilteredCurrent() const {
        return filtered_current_;
    }

}  // namespace driver