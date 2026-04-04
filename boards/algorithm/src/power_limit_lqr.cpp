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

#include <cmath>

namespace control {
namespace {
float ClampFloat(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}
}  // namespace

SeasonRuleManager::SeasonRuleManager(SeasonType season)
    : season_(season),
      Z_max_(60.0f),
      warn_threshold_(0.1f),
      settle_period_(0.1f),
      speed_scale_max_(1.0f),
      current_power_limit_(0.0f),
      Z_(60.0f),
      warn_flag_(false),
      warn_type_(WARN_NONE),
      speed_scale_(1.0f) {
    InitSeasonParams();
    Z_ = Z_max_;
}

void SeasonRuleManager::InitSeasonParams() {
    switch (season_) {
        case SEASON_2026:
            Z_max_ = 60.0f;
            settle_period_ = 0.1f;
            warn_threshold_ = 0.10f;
            speed_scale_max_ = 1.0f;
            break;
        case SEASON_2025:
            Z_max_ = 60.0f;
            settle_period_ = 0.1f;
            warn_threshold_ = 0.15f;
            speed_scale_max_ = 1.0f;
            break;
        default:
            Z_max_ = 60.0f;
            settle_period_ = 0.1f;
            warn_threshold_ = 0.10f;
            speed_scale_max_ = 1.0f;
            break;
    }
}

void SeasonRuleManager::CalcBufferConsume(float measured_power,
                                          float power_limit, float dt) {
    if (!std::isfinite(measured_power) || !std::isfinite(power_limit) ||
        !std::isfinite(dt) || power_limit <= 0.0f || dt <= 0.0f) {
        return;
    }

    current_power_limit_ = power_limit;

    // RM 2026 rule: Z = Z - (Pr - Pl) * 0.1, evaluated every 100 ms.
    // The same expression also restores the buffer whenever Pr < Pl.
    Z_ -= (measured_power - power_limit) * dt;
    Z_ = ClampFloat(Z_, 0.0f, Z_max_);
}

void SeasonRuleManager::SyncBuffer(float buffer_joule) {
    if (!std::isfinite(buffer_joule)) return;
    Z_ = ClampFloat(buffer_joule, 0.0f, Z_max_);
}

void SeasonRuleManager::JudgeWarn(float measured_power,
                                  float predicted_power,
                                  float power_limit) {
    warn_flag_ = false;
    warn_type_ = WARN_NONE;

    if (!std::isfinite(power_limit) || power_limit <= 0.0f) {
        warn_flag_ = true;
        warn_type_ = WARN_IMMEDIATE;
        speed_scale_ = 0.0f;
        return;
    }

    current_power_limit_ = power_limit;

    if (Z_ <= Z_max_ * warn_threshold_) {
        warn_flag_ = true;
        warn_type_ = WARN_BUFFER;
    }

    if (std::isfinite(predicted_power) && predicted_power > power_limit) {
        warn_flag_ = true;
        warn_type_ = WARN_PREDICT;
    }

    if (std::isfinite(measured_power) && measured_power > power_limit) {
        warn_flag_ = true;
        warn_type_ = WARN_IMMEDIATE;
    }

    CalcSpeedScale(measured_power, power_limit);
}

void SeasonRuleManager::JudgeWarn(float measured_power,
                                  float predicted_power,
                                  bool blood_priority) {
    (void)blood_priority;
    JudgeWarn(measured_power, predicted_power, current_power_limit_);
}

void SeasonRuleManager::CalcSpeedScale(float measured_power,
                                       float power_limit) {
    if (!std::isfinite(measured_power) || measured_power <= 1e-6f) {
        speed_scale_ = 1.0f;
        return;
    }

    speed_scale_ = power_limit / measured_power;
    speed_scale_ = ClampFloat(speed_scale_, 0.1f, speed_scale_max_);
}

float SeasonRuleManager::GetSpeedScale() const { return speed_scale_; }
bool SeasonRuleManager::IsWarning() const { return warn_flag_; }
SeasonRuleManager::WarnType SeasonRuleManager::GetWarnType() const { return warn_type_; }
float SeasonRuleManager::GetBuffer() const { return Z_; }
float SeasonRuleManager::GetPowerLimit(bool blood_priority) const {
    (void)blood_priority;
    return current_power_limit_;
}
float SeasonRuleManager::GetBufferMax() const { return Z_max_; }
float SeasonRuleManager::GetSettlePeriod() const { return settle_period_; }

void SeasonRuleManager::Reset() {
    Z_ = Z_max_;
    current_power_limit_ = 0.0f;
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
    : LQRPowerLimit(Config()) {}

LQRPowerLimit::LQRPowerLimit(const Config& config)
    : kf_estimator_(config.dt),
      rule_manager_(config.season),
      config_(config),
      enabled_(false),
      referee_online_(false),
      max_watt_(0.0f),
      voltage_(config.battery_voltage),
      measured_power_(-1.0f),
      external_buffer_joule_(60.0f),
      requested_power_(0.0f),
      limited_power_estimate_(0.0f),
      model_correction_(1.0f),
      current_scale_(1.0f),
      last_correction_sample_power_(-1.0f),
      correction_sample_initialized_(false),
      settle_power_sum_(0.0f),
      settle_sample_count_(0),
      local_trip_remaining_steps_(0) {}

LQRPowerLimit::~LQRPowerLimit() = default;

void LQRPowerLimit::UpdatePowerState(bool enabled, float max_watt,
                                     float voltage, float buffer_joule,
                                     float measured_power,
                                     bool referee_online) {
    const bool source_changed = referee_online_ != referee_online;
    enabled_ = enabled;
    referee_online_ = referee_online;
    max_watt_ = std::isfinite(max_watt) && max_watt > 0.0f ? max_watt : 0.0f;
    if (std::isfinite(voltage) && voltage > 1.0f) voltage_ = voltage;

    external_buffer_joule_ =
        std::isfinite(buffer_joule)
            ? ClampFloat(buffer_joule, 0.0f, rule_manager_.GetBufferMax())
            : 0.0f;
    measured_power_ = std::isfinite(measured_power) && measured_power >= 0.0f
                          ? measured_power
                          : -1.0f;

    if (referee_online_) {
        rule_manager_.SyncBuffer(external_buffer_joule_);
        local_trip_remaining_steps_ = 0;
        if (source_changed) {
            model_correction_ = 1.0f;
            last_correction_sample_power_ = -1.0f;
            correction_sample_initialized_ = false;
        }
    } else if (source_changed) {
        // Referee dropout enters the explicit local fallback.  Start from the
        // local buffer value sent by Atlas instead of retaining stale official
        // buffer/correction state.
        rule_manager_.SyncBuffer(external_buffer_joule_);
        model_correction_ = 1.0f;
        last_correction_sample_power_ = -1.0f;
        correction_sample_initialized_ = false;
        settle_power_sum_ = 0.0f;
        settle_sample_count_ = 0;
        local_trip_remaining_steps_ = 0;
    }

    // The gimbal board forwards referee data at 10 Hz.  The motor output
    // loop calls this setter at 1 kHz with the same stored sample, so update the
    // model only once per distinct referee measurement instead of integrating
    // one stale sample hundreds of times.
    const bool new_correction_sample =
        referee_online_ && measured_power_ >= 0.0f &&
        (!correction_sample_initialized_ ||
         std::fabs(measured_power_ - last_correction_sample_power_) > 0.05f);
    if (new_correction_sample) {
        UpdateModelCorrection();
        last_correction_sample_power_ = measured_power_;
        correction_sample_initialized_ = true;
    }

    if (!enabled_) current_scale_ = 1.0f;
}

void LQRPowerLimit::UpdatePowerState(bool enabled, float max_watt,
                                     float voltage,
                                     uint8_t buffer_percent) {
    const float buffer_joule =
        rule_manager_.GetBufferMax() * static_cast<float>(buffer_percent) / 100.0f;
    UpdatePowerState(enabled, max_watt, voltage, buffer_joule, -1.0f, false);
}

void LQRPowerLimit::UpdateModelCorrection() {
    if (!referee_online_ || measured_power_ < MIN_CORRECTION_POWER_W ||
        limited_power_estimate_ < MIN_CORRECTION_POWER_W) {
        return;
    }

    const float uncorrected_model_power =
        limited_power_estimate_ / model_correction_;
    if (uncorrected_model_power < MIN_CORRECTION_POWER_W) return;

    float ratio = measured_power_ / uncorrected_model_power;
    ratio = ClampFloat(ratio, MODEL_CORRECTION_MIN, MODEL_CORRECTION_MAX);
    model_correction_ += MODEL_CORRECTION_ALPHA * (ratio - model_correction_);
    model_correction_ = ClampFloat(model_correction_, MODEL_CORRECTION_MIN,
                                   MODEL_CORRECTION_MAX);
}

void LQRPowerLimit::UpdateMotorData(const float* omega,
                                    const int16_t* current,
                                    uint8_t motor_num) {
    (void)omega;
    if (current == nullptr || motor_num == 0) {
        requested_power_ = 0.0f;
        return;
    }

    requested_power_ = CalcRequestedPower(current, motor_num);
    kf_estimator_.Update(requested_power_);
}

float LQRPowerLimit::GetControlPowerBudget() const {
    if (max_watt_ <= 0.0f) return 0.0f;

    float margin = referee_online_ ? config_.referee_safety_margin
                                   : config_.local_safety_margin;
    margin = ClampFloat(margin, 0.10f, 1.0f);

    // When official buffer is nearly empty, leave more margin for sampling
    // delay and model error.  This does not invent extra power; it only tightens.
    const float buffer = rule_manager_.GetBuffer();
    if (buffer <= 5.0f) {
        margin = margin < 0.75f ? margin : 0.75f;
    } else if (buffer <= 15.0f) {
        margin = margin < 0.82f ? margin : 0.82f;
    } else if (buffer <= 30.0f) {
        margin = margin < 0.88f ? margin : 0.88f;
    }

    return max_watt_ * margin;
}

void LQRPowerLimit::ApplyLimit(int16_t* turn_current,
                               const float* angular_velocity,
                               uint8_t motor_num) {
    if (!enabled_ || turn_current == nullptr || angular_velocity == nullptr ||
        motor_num == 0) {
        return;
    }

    const uint8_t count = motor_num > 4 ? 4 : motor_num;

    if (local_trip_remaining_steps_ > 0) {
        --local_trip_remaining_steps_;
        current_scale_ = 0.0f;
        limited_power_estimate_ = 0.0f;
        for (uint8_t i = 0; i < count; ++i) turn_current[i] = 0;
        SettleLocalRule(0.0f);
        return;
    }

    const float power_budget = GetControlPowerBudget();
    if (voltage_ <= 1.0f || power_budget <= 0.0f) {
        current_scale_ = 0.0f;
        limited_power_estimate_ = 0.0f;
        for (uint8_t i = 0; i < count; ++i) turn_current[i] = 0;
        SettleLocalRule(0.0f);
        return;
    }

    int16_t original_current[4] = {0, 0, 0, 0};
    for (uint8_t i = 0; i < count; ++i) original_current[i] = turn_current[i];

    float target_scale = 1.0f;
    const float full_power = PredictCorrectedPower(
        original_current, angular_velocity, count, 1.0f);

    if (full_power > power_budget) {
        const float zero_command_power = PredictCorrectedPower(
            original_current, angular_velocity, count, 0.0f);

        if (zero_command_power >= power_budget) {
            target_scale = 0.0f;
        } else {
            float low = 0.0f;
            float high = 1.0f;
            for (uint8_t iteration = 0;
                 iteration < CURRENT_SCALE_SEARCH_ITERATIONS; ++iteration) {
                const float middle = (low + high) * 0.5f;
                const float predicted_power = PredictCorrectedPower(
                    original_current, angular_velocity, count, middle);
                if (predicted_power <= power_budget) {
                    low = middle;
                } else {
                    high = middle;
                }
            }
            target_scale = low;
        }
    }

    // Clamp immediately; release slowly to avoid oscillation at the boundary.
    if (target_scale < current_scale_) {
        current_scale_ = target_scale;
    } else {
        current_scale_ += CURRENT_SCALE_RELEASE_STEP;
        if (current_scale_ > target_scale) current_scale_ = target_scale;
    }
    current_scale_ = ClampFloat(current_scale_, 0.0f, 1.0f);

    for (uint8_t i = 0; i < count; ++i) {
        turn_current[i] = static_cast<int16_t>(std::lround(
            static_cast<float>(original_current[i]) * current_scale_));
    }

    limited_power_estimate_ = PredictCorrectedPower(
        original_current, angular_velocity, count, current_scale_);
    SettleLocalRule(limited_power_estimate_);

    const float power_for_warning =
        referee_online_ && measured_power_ >= 0.0f
            ? measured_power_
            : limited_power_estimate_;
    rule_manager_.JudgeWarn(power_for_warning,
                            limited_power_estimate_, max_watt_);
}

void LQRPowerLimit::SettleLocalRule(float limited_power) {
    if (referee_online_) {
        rule_manager_.SyncBuffer(external_buffer_joule_);
        return;
    }

    settle_power_sum_ += limited_power;
    ++settle_sample_count_;
    if (settle_sample_count_ < SETTLE_BUFFER_SIZE) return;

    const float average_power =
        settle_power_sum_ / static_cast<float>(settle_sample_count_);
    settle_power_sum_ = 0.0f;
    settle_sample_count_ = 0;

    rule_manager_.CalcBufferConsume(average_power, max_watt_,
                                    rule_manager_.GetSettlePeriod());

    if (rule_manager_.GetBuffer() <= 0.0f && average_power > max_watt_) {
        const float steps = LOCAL_TRIP_SECONDS / config_.dt;
        local_trip_remaining_steps_ =
            steps > 0.0f ? static_cast<uint32_t>(steps) : 1U;
    }
}

float LQRPowerLimit::CalcRequestedPower(const int16_t* current,
                                        uint8_t motor_num) const {
    float total_current = 0.0f;
    const uint8_t count = motor_num > 4 ? 4 : motor_num;
    for (uint8_t i = 0; i < count; ++i) {
        total_current += std::fabs(static_cast<float>(current[i]));
    }
    const float current_amp = total_current / 16384.0f * 20.0f;
    return voltage_ * current_amp;
}

float LQRPowerLimit::PowerModel(float angular_velocity,
                                float turn_current) const {
    return K1 * angular_velocity * turn_current +
           K2 * angular_velocity * angular_velocity +
           K3 * turn_current * turn_current + K4;
}

float LQRPowerLimit::PredictTotalEquivalentCurrent(
    const int16_t* turn_current, const float* angular_velocity,
    uint8_t motor_num, float common_scale) const {
    float total = 0.0f;
    const uint8_t count = motor_num > 4 ? 4 : motor_num;
    for (uint8_t i = 0; i < count; ++i) {
        const float scaled_current =
            static_cast<float>(turn_current[i]) * common_scale;
        total += std::fabs(PowerModel(angular_velocity[i], scaled_current));
    }
    return total;
}

float LQRPowerLimit::PredictCorrectedPower(
    const int16_t* turn_current, const float* angular_velocity,
    uint8_t motor_num, float common_scale) const {
    const float equivalent_current_ma = PredictTotalEquivalentCurrent(
        turn_current, angular_velocity, motor_num, common_scale);
    return equivalent_current_ma * voltage_ / 1000.0f * model_correction_;
}

float LQRPowerLimit::GetSpeedScale() const { return rule_manager_.GetSpeedScale(); }
float LQRPowerLimit::GetPowerEstimate() const { return kf_estimator_.GetEstimate(); }
float LQRPowerLimit::GetPowerPrediction() const { return kf_estimator_.GetPrediction(); }
float LQRPowerLimit::GetLimitedPowerEstimate() const { return limited_power_estimate_; }
float LQRPowerLimit::GetMeasuredPower() const { return measured_power_; }
float LQRPowerLimit::GetPowerLimit() const { return max_watt_; }
float LQRPowerLimit::GetModelCorrection() const { return model_correction_; }
float LQRPowerLimit::GetCurrentScale() const { return current_scale_; }
float LQRPowerLimit::GetBuffer() const { return rule_manager_.GetBuffer(); }
bool LQRPowerLimit::IsRefereeSource() const { return referee_online_; }
bool LQRPowerLimit::IsLocalTripActive() const { return local_trip_remaining_steps_ > 0; }
bool LQRPowerLimit::IsWarning() const { return rule_manager_.IsWarning(); }
SeasonRuleManager::WarnType LQRPowerLimit::GetWarnType() const {
    return rule_manager_.GetWarnType();
}

void LQRPowerLimit::Reset() {
    kf_estimator_.Reset();
    rule_manager_.Reset();
    referee_online_ = false;
    measured_power_ = -1.0f;
    external_buffer_joule_ = rule_manager_.GetBufferMax();
    requested_power_ = 0.0f;
    limited_power_estimate_ = 0.0f;
    model_correction_ = 1.0f;
    current_scale_ = 1.0f;
    last_correction_sample_power_ = -1.0f;
    correction_sample_initialized_ = false;
    settle_power_sum_ = 0.0f;
    settle_sample_count_ = 0;
    local_trip_remaining_steps_ = 0;
}

void LQRPowerLimit::SetBloodPriority(bool blood_priority) {
    config_.blood_priority = blood_priority;
}

void LQRPowerLimit::SetSeason(SeasonRuleManager::SeasonType season) {
    config_.season = season;
    rule_manager_.SetSeason(season);
}

}  // namespace control
