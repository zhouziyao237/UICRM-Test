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

namespace control {

    KFPowerEstimator::KFPowerEstimator(float dt) : dt_(dt), P_est_(0), P_pred_(0) {
        A_[0][0] = 1.0f; A_[0][1] = dt_;
        A_[1][0] = 0.0f; A_[1][1] = 1.0f;
        
        H_[0] = 1.0f;
        H_[1] = 0.0f;
        
        Q_[0][0] = 0.1f;  Q_[0][1] = 0.0f;
        Q_[1][0] = 0.0f;  Q_[1][1] = 0.01f;
        
        R_ = 1.0f;
        
        P_[0][0] = 1.0f;  P_[0][1] = 0.0f;
        P_[1][0] = 0.0f;  P_[1][1] = 1.0f;
        
        x_[0] = 0.0f;
        x_[1] = 0.0f;
    }
    
    void KFPowerEstimator::Predict() {
        float x_pred[2];
        x_pred[0] = A_[0][0] * x_[0] + A_[0][1] * x_[1];
        x_pred[1] = A_[1][0] * x_[0] + A_[1][1] * x_[1];

        float P_pred_mat[2][2];
        P_pred_mat[0][0] = A_[0][0] * (A_[0][0] * P_[0][0] + A_[0][1] * P_[1][0]) +
                           A_[0][1] * (A_[0][0] * P_[0][1] + A_[0][1] * P_[1][1]) + Q_[0][0];
        P_pred_mat[0][1] = A_[0][0] * (A_[1][0] * P_[0][0] + A_[1][1] * P_[1][0]) +
                           A_[0][1] * (A_[1][0] * P_[0][1] + A_[1][1] * P_[1][1]) + Q_[0][1];
        P_pred_mat[1][0] = A_[1][0] * (A_[0][0] * P_[0][0] + A_[0][1] * P_[1][0]) +
                           A_[1][1] * (A_[0][0] * P_[0][1] + A_[0][1] * P_[1][1]) + Q_[1][0];
        P_pred_mat[1][1] = A_[1][0] * (A_[1][0] * P_[0][0] + A_[1][1] * P_[1][0]) +
                           A_[1][1] * (A_[1][0] * P_[0][1] + A_[1][1] * P_[1][1]) + Q_[1][1];

        x_[0] = x_pred[0];
        x_[1] = x_pred[1];
        P_[0][0] = P_pred_mat[0][0];
        P_[0][1] = P_pred_mat[0][1];
        P_[1][0] = P_pred_mat[1][0];
        P_[1][1] = P_pred_mat[1][1];
        P_est_ = x_[0];
        P_pred_ = x_[0] + 0.1f * x_[1];
    }

    void KFPowerEstimator::Correct(float P_meas, float measurement_noise) {
        if (measurement_noise > 0.0f) R_ = measurement_noise;

        const float y = P_meas - (H_[0] * x_[0] + H_[1] * x_[1]);
        const float S = H_[0] * (H_[0] * P_[0][0] + H_[1] * P_[1][0]) +
                        H_[1] * (H_[0] * P_[0][1] + H_[1] * P_[1][1]) + R_;
        if (S <= 1e-9f) return;

        float K[2];
        K[0] = (P_[0][0] * H_[0] + P_[0][1] * H_[1]) / S;
        K[1] = (P_[1][0] * H_[0] + P_[1][1] * H_[1]) / S;

        const float p00 = P_[0][0];
        const float p01 = P_[0][1];
        const float p10 = P_[1][0];
        const float p11 = P_[1][1];

        x_[0] += K[0] * y;
        x_[1] += K[1] * y;

        P_[0][0] = (1.0f - K[0] * H_[0]) * p00 - K[0] * H_[1] * p10;
        P_[0][1] = (1.0f - K[0] * H_[0]) * p01 - K[0] * H_[1] * p11;
        P_[1][0] = -K[1] * H_[0] * p00 + (1.0f - K[1] * H_[1]) * p10;
        P_[1][1] = -K[1] * H_[0] * p01 + (1.0f - K[1] * H_[1]) * p11;

        P_est_ = x_[0];
        P_pred_ = x_[0] + 0.1f * x_[1];
    }

    void KFPowerEstimator::Update(float P_meas) {
        Predict();
        Correct(P_meas, R_);
    }

    float KFPowerEstimator::GetEstimate() const {
        return P_est_;
    }
    
    float KFPowerEstimator::GetPrediction() const {
        return P_pred_;
    }

    float KFPowerEstimator::GetRate() const {
        return x_[1];
    }

    float KFPowerEstimator::PredictAhead(float horizon_s) const {
        if (horizon_s < 0.0f) horizon_s = 0.0f;
        return x_[0] + horizon_s * x_[1];
    }
    
    void KFPowerEstimator::Reset() {
        x_[0] = 0.0f;
        x_[1] = 0.0f;
        P_[0][0] = 1.0f;  P_[0][1] = 0.0f;
        P_[1][0] = 0.0f;  P_[1][1] = 1.0f;
        P_est_ = 0.0f;
        P_pred_ = 0.0f;
    }
    
    void KFPowerEstimator::SetProcessNoise(float q0, float q1) {
        Q_[0][0] = q0;
        Q_[1][1] = q1;
    }
    
    void KFPowerEstimator::SetMeasureNoise(float r) {
        R_ = r;
    }

}  // namespace control
