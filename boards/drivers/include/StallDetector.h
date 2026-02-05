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

#pragma once

#include <stdint.h>
#include "KalmanFilter.h"

namespace driver {

    /**
     * @brief 堵转检测配置结构体
     */
    struct StallConfig {
        float current_ratio_threshold;   // 电流比例阈值 (实际电流/目标电流)
        uint32_t detection_time_ms;      // 检测时间窗口 (ms)
        float min_target_current;        // 最小目标电流阈值 (低于此值不检测)
        float speed_threshold;           // 速度阈值 (rad/s，低于此值认为低速)

        // 卡尔曼滤波器配置
        control::KalmanConfig speed_kf_config;
        control::KalmanConfig current_kf_config;
    };

    /**
     * @brief 堵转检测器类
     */
    class StallDetector {
      public:
        /**
         * @brief 构造函数
         */
        StallDetector();

        /**
         * @brief 初始化堵转检测器
         * @param config 堵转检测配置
         */
        void Init(const StallConfig& config);

        /**
         * @brief 更新电机状态
         * @param actual_current 实际电流 (mA)
         * @param target_current 目标电流 (mA)
         * @param actual_speed 实际速度 (rad/s)
         * @param timestamp 时间戳 (ms)
         */
        void Update(float actual_current, float target_current, float actual_speed, uint32_t timestamp);

        /**
         * @brief 检查是否堵转
         * @return true 如果电机处于堵转状态
         */
        bool IsStalled() const;

        /**
         * @brief 获取堵转持续时间
         * @return 堵转持续时间 (ms)
         */
        uint32_t GetStallDuration() const;

        /**
         * @brief 重置堵转检测器
         */
        void Reset();

        /**
         * @brief 获取滤波后的速度
         * @return 滤波后的速度 (rad/s)
         */
        float GetFilteredSpeed() const;

        /**
         * @brief 获取滤波后的电流
         * @return 滤波后的电流 (mA)
         */
        float GetFilteredCurrent() const;

      private:
        StallConfig config_;

        // 卡尔曼滤波器
        control::KalmanFilter speed_filter_;
        control::KalmanFilter current_filter_;

        // 滤波后的值
        float filtered_speed_;
        float filtered_current_;

        // 堵转检测状态
        bool is_stalled_;
        uint32_t stall_start_time_;
        uint32_t last_update_time_;

        /**
         * @brief 检查堵转条件
         * @param filtered_current 滤波后的电流
         * @param target_current 目标电流
         * @param filtered_speed 滤波后的速度
         * @return true 如果满足堵转条件
         */
        bool CheckStallCondition(float filtered_current, float target_current, float filtered_speed);
    };

}  // namespace driver