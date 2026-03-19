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

namespace control {

    /**
     * @brief 卡尔曼滤波器配置结构体
     */
    struct KalmanConfig {
        float process_noise;      // 过程噪声协方差 Q
        float measurement_noise;  // 测量噪声协方差 R
        float initial_estimate;   // 初始估计值
        float initial_error;      // 初始误差协方差 P
    };

    /**
     * @brief 简化的卡尔曼滤波器类
     */
    class KalmanFilter {
    public:
        /**
         * @brief 构造函数
         * @param config 卡尔曼滤波器配置
         */
        explicit KalmanFilter(const KalmanConfig& config);

        /**
         * @brief 默认构造函数
         */
        KalmanFilter();

        /**
         * @brief 初始化滤波器
         * @param config 滤波器配置
         */
        void Init(const KalmanConfig& config);

        /**
         * @brief 重置滤波器
         */
        void Reset();

        /**
         * @brief 更新滤波器状态
         * @param measurement 测量值
         * @return 滤波后的估计值
         */
        float Update(float measurement);

        /**
         * @brief 获取当前估计值
         * @return 当前估计值
         */
        float GetEstimate() const;

        /**
         * @brief 获取当前误差协方差
         * @return 当前误差协方差
         */
        float GetErrorCovariance() const;

    private:
        float x_;  // 状态估计值
        float p_;  // 估计误差协方差
        float q_;  // 过程噪声协方差
        float r_;  // 测量噪声协方差
        float k_;  // 卡尔曼增益
    };

}  // namespace control

namespace control {

    class KFPowerEstimator {
    public:
        explicit KFPowerEstimator(float dt = 0.001f);
        
        void Update(float P_meas);
        
        float GetEstimate() const;
        
        float GetPrediction() const;
        
        void Reset();
        
        void SetProcessNoise(float q0, float q1);
        
        void SetMeasureNoise(float r);

    private:
        float A_[2][2];
        float H_[2];
        float Q_[2][2];
        float R_;
        float P_[2][2];
        float x_[2];
        float dt_;
        float P_est_;
        float P_pred_;
    };

}  // namespace control
