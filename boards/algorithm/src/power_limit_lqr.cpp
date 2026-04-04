/*###########################################################
 # Copyright (c) 2026. BNU-HKBU UIC RoboMaster              #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "power_limit_lqr.h"
#include "utils.h"
#include <cmath>
#include <cstring>

namespace control {

    SeasonRuleManager::SeasonRuleManager(SeasonType season) 
        : season_(season), Z_(0), warn_flag_(false), warn_type_(WARN_NONE), speed_scale_(1.0f) {
        InitSeasonParams();
        Z_ = Z_max_;
    }
    
    void SeasonRuleManager::InitSeasonParams() {
        switch (season_) {
            case SEASON_2026:
                P_limit_blood_ = 75.0f;
                P_limit_power_ = 90.0f;
                Z_max_ = 60.0f;
                warn_threshold_ = 0.1f;
                settle_period_ = 0.1f;
                speed_scale_max_ = 1.2f;
                break;
            case SEASON_2025:
                P_limit_blood_ = 60.0f;
                P_limit_power_ = 80.0f;
                Z_max_ = 50.0f;
                warn_threshold_ = 0.15f;
                settle_period_ = 0.2f;
                speed_scale_max_ = 1.1f;
                break;
            default:
                P_limit_blood_ = 75.0f;
                P_limit_power_ = 90.0f;
                Z_max_ = 60.0f;
                warn_threshold_ = 0.1f;
                settle_period_ = 0.1f;
                speed_scale_max_ = 1.2f;
                break;
        }
    }
    
    void SeasonRuleManager::CalcBufferConsume(float P_settle, float P_limit, float dt) {
        if (P_settle > P_limit) {
            Z_ = Z_ - (P_settle - P_limit) * dt;
        }
        
        if (season_ == SEASON_2025 && P_settle < P_limit) {
            Z_ = Z_ + (P_limit - P_settle) * dt * 0.5f;
        }
        
        if (Z_ > Z_max_) Z_ = Z_max_;
        if (Z_ < 0) Z_ = 0;
    }
    
    void SeasonRuleManager::JudgeWarn(float P_meas, float P_pred, bool blood_priority) {
        float P_limit = GetPowerLimit(blood_priority);
        
        warn_flag_ = false;
        warn_type_ = WARN_NONE;
        
        if (Z_ < Z_max_ * warn_threshold_) {
            warn_flag_ = true;
            warn_type_ = WARN_BUFFER;
        }
        
        if (P_pred > P_limit * 1.05f) {
            warn_flag_ = true;
            warn_type_ = WARN_PREDICT;
        }
        
        if (P_meas > P_limit) {
            warn_flag_ = true;
            warn_type_ = WARN_IMMEDIATE;
        }
        
        CalcSpeedScale(P_meas, P_limit);
    }
    
    void SeasonRuleManager::CalcSpeedScale(float P_meas, float P_limit) {
        if (P_meas < 1e-6f) {
            speed_scale_ = 1.0f;
            return;
        }
        
        switch (warn_type_) {
            case WARN_NONE:
                speed_scale_ = speed_scale_max_;
                if (P_meas > 0) {
                    float scale = P_limit / P_meas;
                    if (scale < speed_scale_) speed_scale_ = scale;
                }
                break;
            case WARN_BUFFER:
                speed_scale_ = 0.5f;
                if (P_meas > 0) {
                    float scale = P_limit / P_meas;
                    if (scale < speed_scale_) speed_scale_ = scale;
                }
                break;
            case WARN_PREDICT:
                speed_scale_ = 0.7f;
                if (P_meas > 0) {
                    float scale = P_limit / P_meas;
                    if (scale < speed_scale_) speed_scale_ = scale;
                }
                break;
            case WARN_IMMEDIATE:
                speed_scale_ = 0.3f;
                if (P_meas > 0) {
                    float scale = P_limit / P_meas;
                    if (scale < speed_scale_) speed_scale_ = scale;
                }
                break;
        }
        
        if (speed_scale_ < 0.1f) speed_scale_ = 0.1f;
        if (speed_scale_ > speed_scale_max_) speed_scale_ = speed_scale_max_;
    }
    
    float SeasonRuleManager::GetSpeedScale() const {
        return speed_scale_;
    }
    
    bool SeasonRuleManager::IsWarning() const {
        return warn_flag_;
    }
    
    SeasonRuleManager::WarnType SeasonRuleManager::GetWarnType() const {
        return warn_type_;
    }
    
    float SeasonRuleManager::GetBuffer() const {
        return Z_;
    }
    
    float SeasonRuleManager::GetPowerLimit(bool blood_priority) const {
        return blood_priority ? P_limit_blood_ : P_limit_power_;
    }
    
    float SeasonRuleManager::GetBufferMax() const {
        return Z_max_;
    }
    
    float SeasonRuleManager::GetSettlePeriod() const {
        return settle_period_;
    }
    
    void SeasonRuleManager::Reset() {
        Z_ = Z_max_;
        warn_flag_ = false;
        warn_type_ = WARN_NONE;
        speed_scale_ = 1.0f;
    }
    
    void SeasonRuleManager::SetSeason(SeasonType season) {
        season_ = season;
        InitSeasonParams();
        Reset();
    }

    LQRPowerLimit::LQRPowerLimit() 
        : kf_estimator_(0.001f)
        , rule_manager_(SeasonRuleManager::SEASON_2026)
        , config_()
        , enabled_(false)
        , max_watt_(0)
        , voltage_(24.0f)
        , current_power_(0)
        , buffer_percent_(0)
        , power_buffer_idx_(0)
        , power_sum_(0)
        , power_count_(0) {
        memset(power_buffer_, 0, sizeof(power_buffer_));
    }
    
    LQRPowerLimit::LQRPowerLimit(const Config& config) 
        : kf_estimator_(config.dt)
        , rule_manager_(config.season)
        , config_(config)
        , enabled_(false)
        , max_watt_(0)
        , voltage_(config.battery_voltage)
        , current_power_(0)
        , buffer_percent_(0)
        , power_buffer_idx_(0)
        , power_sum_(0)
        , power_count_(0) {
        memset(power_buffer_, 0, sizeof(power_buffer_));
    }
    
    LQRPowerLimit::~LQRPowerLimit() {
    }
    
    void LQRPowerLimit::UpdatePowerState(bool enabled, float max_watt, float voltage, uint8_t buffer_percent) {
        enabled_ = enabled;
        max_watt_ = max_watt;
        if (voltage > 1.0f) {
            voltage_ = voltage;
        }
        buffer_percent_ = buffer_percent;
    }
    
    void LQRPowerLimit::UpdateMotorData(const float* omega, const int16_t* current, uint8_t motor_num) {
        UNUSED(omega);
        current_power_ = CalcInstantPower(current, motor_num);
        
        kf_estimator_.Update(current_power_);
        
        UpdateSettleBuffer(current_power_);
        
        if (power_count_ >= SETTLE_BUFFER_SIZE) {
            float P_settle = power_sum_ / SETTLE_BUFFER_SIZE;
            float P_limit = rule_manager_.GetPowerLimit(config_.blood_priority);
            rule_manager_.CalcBufferConsume(P_settle, P_limit, config_.dt);
        }
        
        float P_pred = kf_estimator_.GetPrediction();
        rule_manager_.JudgeWarn(current_power_, P_pred, config_.blood_priority);
    }
    
    void LQRPowerLimit::ApplyLimit(int16_t* turn_current, const float* angular_velocity, uint8_t motor_num) {
        if (!enabled_) {
            return;
        }

        float max_current = max_watt_ / voltage_;

        // 计算 buffer 缩放因子（保持不变）
        float buffer_scale = 1.0f;
        if (buffer_percent_ <= 20) {
            buffer_scale = 0.8f;
        } else if (buffer_percent_ <= 50) {
            buffer_scale = 0.8f + (buffer_percent_ - 20) * 0.002f;
        } else if (buffer_percent_ <= 80) {
            buffer_scale = 0.86f + (buffer_percent_ - 50) * 0.00213f;
        } else {
            buffer_scale = 1.5f;
        }
        max_current *= buffer_scale;

        // 计算每个电机的等效电流（用于功率模型）和总绝对值
        float motor_equiv_current[4] = {0};
        float total_abs_equiv = 0;
        for (uint8_t i = 0; i < motor_num && i < 4; i++) {
            motor_equiv_current[i] = PowerModel(angular_velocity[i], turn_current[i]);
            total_abs_equiv += std::abs(motor_equiv_current[i]);
        }

        int16_t max_current_ma = static_cast<int16_t>(max_current * 1000);
        if (total_abs_equiv <= max_current_ma) {
            return;  // 未超限，无需限制
        }

        // 按绝对值比例缩放所有电机
        float ratio = max_current_ma / total_abs_equiv;
        for (uint8_t i = 0; i < motor_num && i < 4; i++) {
            float target_equiv = ratio * motor_equiv_current[i];
            turn_current[i] = ReversePowerModel(angular_velocity[i], turn_current[i], target_equiv);
        }
    }
    
    float LQRPowerLimit::CalcInstantPower(const int16_t* current, uint8_t motor_num) {
        float total_current = 0;
        for (uint8_t i = 0; i < motor_num; i++) {
            total_current += std::abs(current[i]);
        }
        float current_amp = total_current / 16384.0f * 20.0f;
        return voltage_ * current_amp;
    }
    
    float LQRPowerLimit::PowerModel(float angular_velocity, int16_t turn_current) {
        return K1 * angular_velocity * turn_current +
               K2 * angular_velocity * angular_velocity +
               K3 * turn_current * turn_current + K4;
    }
    
    int16_t LQRPowerLimit::ReversePowerModel(float angular_velocity, int16_t turn_current, float target_current) {
        float a = K3;
        float b = K1 * angular_velocity;
        float c = K2 * angular_velocity * angular_velocity + K4 - target_current;
        
        float delta = b * b - 4 * a * c;
        if (delta < 0) {
            return turn_current;
        }
        
        float sqrt_delta = std::sqrt(delta);
        float x1 = (-b + sqrt_delta) / (2 * a);
        float x2 = (-b - sqrt_delta) / (2 * a);
        
        if (std::abs(x1 - turn_current) < std::abs(x2 - turn_current)) {
            return static_cast<int16_t>(x1);
        } else {
            return static_cast<int16_t>(x2);
        }
    }
    
    void LQRPowerLimit::UpdateSettleBuffer(float power) {
        power_sum_ -= power_buffer_[power_buffer_idx_];
        power_buffer_[power_buffer_idx_] = power;
        power_sum_ += power;
        power_buffer_idx_ = (power_buffer_idx_ + 1) % SETTLE_BUFFER_SIZE;
        if (power_count_ < SETTLE_BUFFER_SIZE) {
            power_count_++;
        }
    }
    
    float LQRPowerLimit::GetSpeedScale() const {
        return rule_manager_.GetSpeedScale();
    }
    
    float LQRPowerLimit::GetPowerEstimate() const {
        return kf_estimator_.GetEstimate();
    }
    
    float LQRPowerLimit::GetPowerPrediction() const {
        return kf_estimator_.GetPrediction();
    }
    
    float LQRPowerLimit::GetBuffer() const {
        return rule_manager_.GetBuffer();
    }
    
    bool LQRPowerLimit::IsWarning() const {
        return rule_manager_.IsWarning();
    }
    
    SeasonRuleManager::WarnType LQRPowerLimit::GetWarnType() const {
        return rule_manager_.GetWarnType();
    }
    
    void LQRPowerLimit::Reset() {
        kf_estimator_.Reset();
        rule_manager_.Reset();
        current_power_ = 0;
        power_buffer_idx_ = 0;
        power_sum_ = 0;
        power_count_ = 0;
        memset(power_buffer_, 0, sizeof(power_buffer_));
    }
    
    void LQRPowerLimit::SetBloodPriority(bool blood_priority) {
        config_.blood_priority = blood_priority;
    }
    
    void LQRPowerLimit::SetSeason(SeasonRuleManager::SeasonType season) {
        config_.season = season;
        rule_manager_.SetSeason(season);
    }

}  // namespace control
