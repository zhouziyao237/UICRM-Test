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

float Positive(float value) { return value > 0.0f ? value : 0.0f; }
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

LQRPowerLimit::LQRPowerLimit() : LQRPowerLimit(Config()) {}

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
      raw_model_power_(0.0f),
      requested_power_(0.0f),
      limited_power_estimate_(0.0f),
      power_prediction_(0.0f),
      power_rate_(0.0f),
      model_correction_(1.0f),
      current_scale_(1.0f),
      lqr_scale_(1.0f),
      hard_scale_(1.0f),
      predicted_buffer_(60.0f),
      local_bus_power_(0.0f),
      local_bus_voltage_(0.0f),
      local_bus_current_amp_(0.0f),
      local_bus_age_s_(1000.0f),
      local_bus_valid_(false),
      referee_sample_sequence_(0U),
      last_fused_referee_sequence_(0U),
      referee_sample_initialized_(false),
      last_model_window_power_(0.0f),
      model_window_initialized_(false),
      model_window_sum_(0.0f),
      model_window_count_(0),
      settle_power_sum_(0.0f),
      settle_sample_count_(0),
      settle_sample_target_(100),
      local_trip_remaining_steps_(0),
      guard_snapshots_{},
      guard_snapshot_index_(0U) {
    if (!std::isfinite(config_.dt) || config_.dt <= 0.0f) config_.dt = 0.001f;
    if (!std::isfinite(config_.prediction_horizon_s) ||
        config_.prediction_horizon_s <= 0.0f) {
        config_.prediction_horizon_s = 0.10f;
    }
    if (!std::isfinite(config_.local_bus_timeout_s) ||
        config_.local_bus_timeout_s <= 0.0f) {
        config_.local_bus_timeout_s = 0.020f;
    }

    const float settle_samples = rule_manager_.GetSettlePeriod() / config_.dt;
    settle_sample_target_ = static_cast<uint16_t>(
        ClampFloat(std::lround(settle_samples), 1.0f, 65535.0f));

    kf_estimator_.SetProcessNoise(config_.kf_q_power, config_.kf_q_rate);
    kf_estimator_.SetMeasureNoise(config_.kf_r_model);
    PublishGuardSnapshot();
}

LQRPowerLimit::~LQRPowerLimit() = default;

void LQRPowerLimit::UpdatePowerState(bool enabled, float max_watt,
                                     float voltage, float buffer_joule,
                                     float measured_power,
                                     bool referee_online,
                                     uint32_t referee_sample_sequence) {
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

    referee_sample_sequence_ = referee_sample_sequence;

    if (referee_online_) {
        rule_manager_.SyncBuffer(external_buffer_joule_);
        local_trip_remaining_steps_ = 0;
        if (source_changed) {
            model_correction_ = 1.0f;
            // Force the latest official sample to be fused once after the source
            // becomes online, even when two adjacent Pr samples are numerically equal.
            last_fused_referee_sequence_ = referee_sample_sequence_ - 1U;
            referee_sample_initialized_ = false;
        }
    } else if (source_changed) {
        rule_manager_.SyncBuffer(external_buffer_joule_);
        model_correction_ = 1.0f;
        last_fused_referee_sequence_ = referee_sample_sequence_;
        referee_sample_initialized_ = false;
        settle_power_sum_ = 0.0f;
        settle_sample_count_ = 0;
        local_trip_remaining_steps_ = 0;
    }

    if (!enabled_) {
        current_scale_ = 1.0f;
        lqr_scale_ = 1.0f;
        hard_scale_ = 1.0f;
    }
}

void LQRPowerLimit::UpdatePowerState(bool enabled, float max_watt,
                                     float voltage,
                                     uint8_t buffer_percent) {
    const float buffer_joule =
        rule_manager_.GetBufferMax() * static_cast<float>(buffer_percent) / 100.0f;
    UpdatePowerState(enabled, max_watt, voltage, buffer_joule, -1.0f, false);
}

void LQRPowerLimit::UpdateLocalBusMeasurement(float voltage,
                                              float current_amp,
                                              bool valid) {
    if (!valid || !std::isfinite(voltage) || !std::isfinite(current_amp) ||
        voltage <= 1.0f) {
        InvalidateLocalBusMeasurement();
        return;
    }

    local_bus_voltage_ = voltage;
    local_bus_current_amp_ = current_amp;
    local_bus_power_ = voltage * current_amp;
    local_bus_age_s_ = 0.0f;
    local_bus_valid_ = true;
    voltage_ = voltage;
}

void LQRPowerLimit::InvalidateLocalBusMeasurement() {
    local_bus_valid_ = false;
    local_bus_age_s_ = config_.local_bus_timeout_s + config_.dt;
}

void LQRPowerLimit::UpdateModelCorrection(float truth_power,
                                          float model_power,
                                          float alpha) {
    if (!std::isfinite(truth_power) || !std::isfinite(model_power) ||
        truth_power < MIN_CORRECTION_POWER_W ||
        model_power < MIN_CORRECTION_POWER_W || alpha <= 0.0f) {
        return;
    }

    float ratio = truth_power / model_power;
    ratio = ClampFloat(ratio, MODEL_CORRECTION_MIN, MODEL_CORRECTION_MAX);
    alpha = ClampFloat(alpha, 0.0f, 1.0f);
    model_correction_ += alpha * (ratio - model_correction_);
    model_correction_ = ClampFloat(model_correction_, MODEL_CORRECTION_MIN,
                                   MODEL_CORRECTION_MAX);
}

void LQRPowerLimit::UpdateRefereeModelCorrection() {
    if (!referee_online_ || measured_power_ < MIN_CORRECTION_POWER_W) return;

    float model_reference = 0.0f;
    if (model_window_initialized_) {
        model_reference = last_model_window_power_ / model_correction_;
    } else if (limited_power_estimate_ >= MIN_CORRECTION_POWER_W) {
        model_reference = limited_power_estimate_ / model_correction_;
    }

    UpdateModelCorrection(measured_power_, model_reference,
                          config_.referee_model_correction_alpha);
}

void LQRPowerLimit::UpdateMotorData(const float* omega,
                                    const int16_t* current,
                                    uint8_t motor_num) {
    if (omega == nullptr || current == nullptr || motor_num == 0) {
        raw_model_power_ = 0.0f;
        requested_power_ = 0.0f;
        power_prediction_ = 0.0f;
        power_rate_ = 0.0f;
        return;
    }

    local_bus_age_s_ += config_.dt;
    if (local_bus_age_s_ > config_.local_bus_timeout_s) {
        local_bus_valid_ = false;
    }

    const uint8_t count = motor_num > 4 ? 4 : motor_num;
    raw_model_power_ = PredictRawPower(current, omega, count, 1.0f);
    requested_power_ = raw_model_power_ * model_correction_;

    // One predict step per PowerManager period, then same-time measurement
    // corrections.  The motor model is always the high-rate feed-forward
    // observation.  Until a local V/I sensor is installed, each *new* referee
    // Pr sample is fused once as the real-power correction.
    kf_estimator_.Predict();
    kf_estimator_.Correct(requested_power_, config_.kf_r_model);

    if (local_bus_valid_) {
        const float bus_power_for_filter = Positive(local_bus_power_);
        kf_estimator_.Correct(bus_power_for_filter, config_.kf_r_bus);

        // Future path: local V*I becomes the fast truth source as soon as a
        // chassis current sensor is registered; no algorithm rewrite is needed.
        UpdateModelCorrection(bus_power_for_filter, raw_model_power_,
                              config_.local_model_correction_alpha);
        requested_power_ = raw_model_power_ * model_correction_;
    } else if (referee_online_ && measured_power_ >= 0.0f &&
               (!referee_sample_initialized_ ||
                referee_sample_sequence_ != last_fused_referee_sequence_)) {
        // Referee Pr is slow (~10 Hz), so never feed the same stale sample on
        // every 1 ms control tick.  Sequence gating also accepts equal-valued
        // consecutive official samples.
        kf_estimator_.Correct(measured_power_, config_.kf_r_referee);
        UpdateRefereeModelCorrection();
        last_fused_referee_sequence_ = referee_sample_sequence_;
        referee_sample_initialized_ = true;
    }

    power_rate_ = kf_estimator_.GetRate();
    const float trend_prediction = Positive(
        kf_estimator_.PredictAhead(config_.prediction_horizon_s));

    // Motor command is a feed-forward view of what is about to happen while
    // V*I is the fast truth of what is happening now.  Use the conservative
    // maximum for protection.
    power_prediction_ = trend_prediction > requested_power_
                            ? trend_prediction
                            : requested_power_;

    UpdateDynamicScale();

    // All slow state bookkeeping lives in the PowerManager thread.  The motor
    // pre-output callback is reserved for the final hard guard only.
    model_window_sum_ += limited_power_estimate_;
    ++model_window_count_;
    if (model_window_count_ >= settle_sample_target_) {
        last_model_window_power_ =
            model_window_sum_ / static_cast<float>(model_window_count_);
        model_window_initialized_ = true;
        model_window_sum_ = 0.0f;
        model_window_count_ = 0;
    }

    const float observed_for_local_rule =
        local_bus_valid_ ? Positive(local_bus_power_) : limited_power_estimate_;
    SettleLocalRule(observed_for_local_rule);

    const float power_for_warning =
        local_bus_valid_
            ? Positive(local_bus_power_)
            : (referee_online_ && measured_power_ >= 0.0f
                   ? measured_power_
                   : limited_power_estimate_);
    rule_manager_.JudgeWarn(power_for_warning, power_prediction_, max_watt_);

    if (local_trip_remaining_steps_ > 0U) {
        --local_trip_remaining_steps_;
        lqr_scale_ = 0.0f;
    }
    PublishGuardSnapshot();
}

float LQRPowerLimit::GetControlPowerBudget() const {
    if (max_watt_ <= 0.0f) return 0.0f;

    float margin = referee_online_ ? config_.referee_safety_margin
                                   : config_.local_safety_margin;
    margin = ClampFloat(margin, 0.10f, 1.0f);

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

void LQRPowerLimit::UpdateDynamicScale() {
    if (!enabled_) {
        lqr_scale_ = 1.0f;
        predicted_buffer_ = rule_manager_.GetBuffer();
        return;
    }

    const float budget = GetControlPowerBudget();
    if (budget <= 1e-3f || max_watt_ <= 1e-3f) {
        lqr_scale_ = 0.0f;
        predicted_buffer_ = 0.0f;
        return;
    }

    const float buffer = rule_manager_.GetBuffer();
    const float horizon = config_.prediction_horizon_s;

    // Predict the official-energy trend over the same order of horizon as the
    // 2026 100 ms referee settlement.  This is a local safety estimate only;
    // the referee buffer remains authoritative when online.
    predicted_buffer_ = ClampFloat(
        buffer - (power_prediction_ - max_watt_) * horizon,
        0.0f, rule_manager_.GetBufferMax());

    const float power_error = (power_prediction_ - budget) / budget;
    const float rate_error =
        Positive(power_rate_) * horizon / budget;

    float buffer_error = 0.0f;
    if (config_.lqr_buffer_target_joule > 1e-3f &&
        predicted_buffer_ < config_.lqr_buffer_target_joule) {
        buffer_error =
            (config_.lqr_buffer_target_joule - predicted_buffer_) /
            config_.lqr_buffer_target_joule;
    }

    // Normalized state feedback.  The gains are the vehicle-level LQR tuning
    // surface: power error, future slope, and buffer deficit all reduce the
    // one common four-wheel output scale.  Negative total pressure is not used
    // to boost above 1.0.
    const float pressure = Positive(
        config_.lqr_k_power * power_error +
        config_.lqr_k_rate * rate_error +
        config_.lqr_k_buffer * buffer_error);
    const float desired_scale = ClampFloat(1.0f - pressure, 0.0f, 1.0f);

    const float attack_step = Positive(config_.lqr_attack_per_second) * config_.dt;
    const float release_step = Positive(config_.lqr_release_per_second) * config_.dt;

    if (desired_scale < lqr_scale_) {
        lqr_scale_ -= attack_step;
        if (lqr_scale_ < desired_scale) lqr_scale_ = desired_scale;
    } else {
        lqr_scale_ += release_step;
        if (lqr_scale_ > desired_scale) lqr_scale_ = desired_scale;
    }
    lqr_scale_ = ClampFloat(lqr_scale_, 0.0f, 1.0f);
}

void LQRPowerLimit::ApplyLimit(int16_t* turn_current,
                               const float* angular_velocity,
                               uint8_t motor_num) {
    if (turn_current == nullptr || angular_velocity == nullptr || motor_num == 0) {
        return;
    }

    const GuardSnapshot guard = ReadGuardSnapshot();
    if (!guard.enabled) return;

    const uint8_t count = motor_num > 4 ? 4 : motor_num;

    if (guard.local_trip) {
        current_scale_ = 0.0f;
        hard_scale_ = 0.0f;
        limited_power_estimate_ = 0.0f;
        for (uint8_t i = 0; i < count; ++i) turn_current[i] = 0;
        return;
    }

    const float power_budget = guard.power_budget;
    if (guard.voltage <= 1.0f || power_budget <= 0.0f) {
        current_scale_ = 0.0f;
        hard_scale_ = 0.0f;
        limited_power_estimate_ = 0.0f;
        for (uint8_t i = 0; i < count; ++i) turn_current[i] = 0;
        return;
    }

    int16_t original_current[4] = {0, 0, 0, 0};
    for (uint8_t i = 0; i < count; ++i) original_current[i] = turn_current[i];

    const auto predict_with_guard = [&](float common_scale) {
        const float equivalent_current_ma = PredictTotalEquivalentCurrent(
            original_current, angular_velocity, count, common_scale);
        return equivalent_current_ma * guard.voltage / 1000.0f *
               guard.model_correction;
    };

    // Absolute nonlinear guard runs synchronously in the motor TX path.  The
    // threaded LQR scale may be one tick old; this guard always evaluates the
    // current live motor command before CAN transmit.
    hard_scale_ = 1.0f;
    const float full_power = predict_with_guard(1.0f);

    if (full_power > power_budget) {
        const float zero_command_power = predict_with_guard(0.0f);
        if (zero_command_power >= power_budget) {
            hard_scale_ = 0.0f;
        } else {
            float low = 0.0f;
            float high = 1.0f;
            for (uint8_t iteration = 0;
                 iteration < CURRENT_SCALE_SEARCH_ITERATIONS; ++iteration) {
                const float middle = (low + high) * 0.5f;
                if (predict_with_guard(middle) <= power_budget) {
                    low = middle;
                } else {
                    high = middle;
                }
            }
            hard_scale_ = low;
        }
    }

    current_scale_ = guard.lqr_scale < hard_scale_ ? guard.lqr_scale : hard_scale_;
    current_scale_ = ClampFloat(current_scale_, 0.0f, 1.0f);

    for (uint8_t i = 0; i < count; ++i) {
        turn_current[i] = static_cast<int16_t>(std::lround(
            static_cast<float>(original_current[i]) * current_scale_));
    }

    limited_power_estimate_ = predict_with_guard(current_scale_);
}

void LQRPowerLimit::PublishGuardSnapshot() {
    const uint8_t next = static_cast<uint8_t>(guard_snapshot_index_ ^ 1U);
    guard_snapshots_[next].enabled = enabled_;
    guard_snapshots_[next].local_trip = local_trip_remaining_steps_ > 0U;
    guard_snapshots_[next].power_budget = GetControlPowerBudget();
    guard_snapshots_[next].voltage = voltage_;
    guard_snapshots_[next].model_correction = model_correction_;
    guard_snapshots_[next].lqr_scale = lqr_scale_;
    // Single-byte publish after the inactive buffer is complete. The motor
    // callback never waits on the lower-priority PowerManager thread.
    guard_snapshot_index_ = next;
}

LQRPowerLimit::GuardSnapshot LQRPowerLimit::ReadGuardSnapshot() const {
    const uint8_t index = guard_snapshot_index_ & 0x01U;
    return guard_snapshots_[index];
}

void LQRPowerLimit::SettleLocalRule(float observed_power) {
    if (referee_online_) {
        rule_manager_.SyncBuffer(external_buffer_joule_);
        return;
    }

    settle_power_sum_ += observed_power;
    ++settle_sample_count_;
    if (settle_sample_count_ < settle_sample_target_) return;

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

float LQRPowerLimit::PowerModel(float angular_velocity,
                                float turn_current) const {
    const MotorPowerModel& m = config_.motor_model;
    return m.k1 * angular_velocity * turn_current +
           m.k2 * angular_velocity * angular_velocity +
           m.k3 * turn_current * turn_current + m.k4;
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

float LQRPowerLimit::PredictRawPower(
    const int16_t* turn_current, const float* angular_velocity,
    uint8_t motor_num, float common_scale) const {
    const float equivalent_current_ma = PredictTotalEquivalentCurrent(
        turn_current, angular_velocity, motor_num, common_scale);
    return equivalent_current_ma * voltage_ / 1000.0f;
}

float LQRPowerLimit::PredictCorrectedPower(
    const int16_t* turn_current, const float* angular_velocity,
    uint8_t motor_num, float common_scale) const {
    return PredictRawPower(turn_current, angular_velocity, motor_num,
                           common_scale) * model_correction_;
}

float LQRPowerLimit::GetSpeedScale() const { return rule_manager_.GetSpeedScale(); }
float LQRPowerLimit::GetPowerEstimate() const { return kf_estimator_.GetEstimate(); }
float LQRPowerLimit::GetPowerPrediction() const { return power_prediction_; }
float LQRPowerLimit::GetPowerRate() const { return power_rate_; }
float LQRPowerLimit::GetRequestedModelPower() const { return requested_power_; }
float LQRPowerLimit::GetLimitedPowerEstimate() const { return limited_power_estimate_; }
float LQRPowerLimit::GetMeasuredPower() const { return measured_power_; }
float LQRPowerLimit::GetLocalBusPower() const { return local_bus_power_; }
bool LQRPowerLimit::IsLocalBusMeasurementValid() const { return local_bus_valid_; }
float LQRPowerLimit::GetPowerLimit() const { return max_watt_; }
float LQRPowerLimit::GetControlPowerBudgetPublic() const { return GetControlPowerBudget(); }
float LQRPowerLimit::GetModelCorrection() const { return model_correction_; }
float LQRPowerLimit::GetCurrentScale() const { return current_scale_; }
float LQRPowerLimit::GetLQRScale() const { return lqr_scale_; }
float LQRPowerLimit::GetHardScale() const { return hard_scale_; }
float LQRPowerLimit::GetPredictedBuffer() const { return predicted_buffer_; }
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
    raw_model_power_ = 0.0f;
    requested_power_ = 0.0f;
    limited_power_estimate_ = 0.0f;
    power_prediction_ = 0.0f;
    power_rate_ = 0.0f;
    model_correction_ = 1.0f;
    current_scale_ = 1.0f;
    lqr_scale_ = 1.0f;
    hard_scale_ = 1.0f;
    predicted_buffer_ = rule_manager_.GetBufferMax();
    local_bus_power_ = 0.0f;
    local_bus_voltage_ = 0.0f;
    local_bus_current_amp_ = 0.0f;
    local_bus_age_s_ = 1000.0f;
    local_bus_valid_ = false;
    referee_sample_sequence_ = 0U;
    last_fused_referee_sequence_ = 0U;
    referee_sample_initialized_ = false;
    last_model_window_power_ = 0.0f;
    model_window_initialized_ = false;
    model_window_sum_ = 0.0f;
    model_window_count_ = 0;
    settle_power_sum_ = 0.0f;
    settle_sample_count_ = 0;
    local_trip_remaining_steps_ = 0;
    PublishGuardSnapshot();
}

void LQRPowerLimit::SetBloodPriority(bool blood_priority) {
    config_.blood_priority = blood_priority;
}

void LQRPowerLimit::SetSeason(SeasonRuleManager::SeasonType season) {
    config_.season = season;
    rule_manager_.SetSeason(season);
}

}  // namespace control
