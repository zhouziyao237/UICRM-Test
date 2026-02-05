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

#include "KalmanFilter.h"
#include "utils.h"

namespace control {

    KalmanFilter::KalmanFilter() : x_(0), p_(1.0f), q_(0.1f), r_(0.1f), k_(0) {}

    KalmanFilter::KalmanFilter(const KalmanConfig& config) {
        Init(config);
    }

    void KalmanFilter::Init(const KalmanConfig& config) {
        x_ = config.initial_estimate;
        p_ = config.initial_error;
        q_ = config.process_noise;
        r_ = config.measurement_noise;
        k_ = 0;
    }

    void KalmanFilter::Reset() {
        x_ = 0;
        p_ = 1.0f;
        k_ = 0;
    }

    float KalmanFilter::Update(float measurement) {
        // 预测步骤
        p_ = p_ + q_;

        // 更新步骤
        k_ = p_ / (p_ + r_);
        x_ = x_ + k_ * (measurement - x_);
        p_ = (1 - k_) * p_;

        return x_;
    }

    float KalmanFilter::GetEstimate() const {
        return x_;
    }

    float KalmanFilter::GetErrorCovariance() const {
        return p_;
    }

}  // namespace control
