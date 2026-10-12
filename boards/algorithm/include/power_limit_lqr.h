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

#pragma once

#include <stdint.h>

#include "KalmanFilter.h"

namespace control {

class SeasonRuleManager {
public:
    enum SeasonType { SEASON_2025 = 0, SEASON_2026 = 1 };
    enum WarnType {
        WARN_NONE = 0,
        WARN_BUFFER = 1,
        WARN_PREDICT = 2,
        WARN_IMMEDIATE = 3
    };

    explicit SeasonRuleManager(SeasonType season = SEASON_2026);

    void CalcBufferConsume(float measured_power, float power_limit, float dt);
    void SyncBuffer(float buffer_joule);
    void JudgeWarn(float measured_power, float predicted_power, float power_limit);
    void JudgeWarn(float measured_power, float predicted_power, bool blood_priority);

    float GetSpeedScale() const;
    bool IsWarning() const;
    WarnType GetWarnType() const;
    float GetBuffer() const;
    float GetPowerLimit(bool blood_priority) const;
    float GetBufferMax() const;
    float GetSettlePeriod() const;
    void Reset();
    void SetSeason(SeasonType season);

private:
    void InitSeasonParams();
    void CalcSpeedScale(float measured_power, float power_limit);

    SeasonType season_;
    float Z_max_;
    float warn_threshold_;
    float settle_period_;
    float speed_scale_max_;
    float current_power_limit_;

    float Z_;
    bool warn_flag_;
    WarnType warn_type_;
    float speed_scale_;
};

/**
 * Fast chassis power manager.
 *
 * Roles are deliberately separated:
 *  - motor model: feed-forward power request from omega + motor current command;
 *  - local V*I: fast real measurement when a chassis-bus current sensor exists;
 *  - Kalman: estimate current power and power slope, then predict ahead;
 *  - LQR-style normalized state feedback: continuously changes one common
 *    four-wheel output scale;
 *  - hard guard: nonlinear model + binary search gives an absolute final cap.
 *
 * The referee-system power and buffer remain authoritative for competition
 * state, but they are never the first/only protection because they arrive at a
 * much slower rate than the motor output loop.
 */
class LQRPowerLimit {
public:
    struct MotorPowerModel {
        // M3508/C620 equivalent input-current model, result in mA.
        float k1 = 0.000931f;
        float k2 = 0.000455f;
        float k3 = 0.000006f;
        float k4 = 39.151850f;
    };

    struct Config {
        SeasonRuleManager::SeasonType season;
        bool blood_priority;  // source compatibility only; referee Pl is authoritative.
        float dt;
        float battery_voltage;

        // Absolute control budget.  Buffer thresholds may tighten this further.
        float referee_safety_margin;
        float local_safety_margin;

        // Prediction / local sensor.
        float prediction_horizon_s;
        float local_bus_timeout_s;

        // Kalman tuning.  Local bus V*I is trusted more than the motor model.
        float kf_q_power;
        float kf_q_rate;
        float kf_r_model;
        float kf_r_referee;
        float kf_r_bus;

        // Normalized LQR/state-feedback gains.  Output is one common scale.
        float lqr_k_power;
        float lqr_k_rate;
        float lqr_k_buffer;
        float lqr_buffer_target_joule;
        float lqr_attack_per_second;
        float lqr_release_per_second;

        // Online model adaptation.  Referee is slow/authoritative; local V*I is fast.
        float local_model_correction_alpha;
        float referee_model_correction_alpha;

        MotorPowerModel motor_model;

        Config()
            : season(SeasonRuleManager::SEASON_2026),
              blood_priority(true),
              dt(0.001f),
              battery_voltage(24.0f),
              referee_safety_margin(0.90f),
              local_safety_margin(0.90f),
              prediction_horizon_s(0.10f),
              local_bus_timeout_s(0.020f),
              kf_q_power(0.15f),
              kf_q_rate(0.02f),
              kf_r_model(8.0f),
              kf_r_referee(1.50f),
              kf_r_bus(0.80f),
              lqr_k_power(0.90f),
              lqr_k_rate(0.35f),
              lqr_k_buffer(0.18f),
              lqr_buffer_target_joule(20.0f),
              lqr_attack_per_second(12.0f),
              lqr_release_per_second(2.0f),
              local_model_correction_alpha(0.002f),
              referee_model_correction_alpha(0.05f),
              motor_model() {}
    };

    LQRPowerLimit();
    explicit LQRPowerLimit(const Config& config);
    ~LQRPowerLimit();

    void UpdatePowerState(bool enabled, float max_watt, float voltage,
                          float buffer_joule, float measured_power,
                          bool referee_online,
                          uint32_t referee_sample_sequence = 0U);
    void UpdatePowerState(bool enabled, float max_watt, float voltage,
                          uint8_t buffer_percent);

    /**
     * Feed a fast local chassis-bus measurement.  Current is signed in ampere;
     * negative current (regeneration) is allowed.  The power manager internally
     * uses P = V * I and times the sample out if it stops updating.
     */
    void UpdateLocalBusMeasurement(float voltage, float current_amp, bool valid = true);
    void InvalidateLocalBusMeasurement();

    void UpdateMotorData(const float* omega, const int16_t* current, uint8_t motor_num);
    void ApplyLimit(int16_t* turn_current, const float* angular_velocity, uint8_t motor_num);

    float GetSpeedScale() const;
    float GetPowerEstimate() const;
    float GetPowerPrediction() const;
    float GetPowerRate() const;
    float GetRequestedModelPower() const;
    float GetLimitedPowerEstimate() const;
    float GetMeasuredPower() const;
    float GetLocalBusPower() const;
    bool IsLocalBusMeasurementValid() const;
    float GetPowerLimit() const;
    float GetControlPowerBudgetPublic() const;
    float GetModelCorrection() const;
    float GetCurrentScale() const;
    float GetLQRScale() const;
    float GetHardScale() const;
    float GetPredictedBuffer() const;
    float GetBuffer() const;
    bool IsRefereeSource() const;
    bool IsLocalTripActive() const;
    bool IsWarning() const;
    SeasonRuleManager::WarnType GetWarnType() const;

    void Reset();
    void SetBloodPriority(bool blood_priority);
    void SetSeason(SeasonRuleManager::SeasonType season);

private:
    float PowerModel(float angular_velocity, float turn_current) const;
    float PredictTotalEquivalentCurrent(const int16_t* turn_current,
                                        const float* angular_velocity,
                                        uint8_t motor_num,
                                        float common_scale) const;
    float PredictRawPower(const int16_t* turn_current,
                          const float* angular_velocity,
                          uint8_t motor_num,
                          float common_scale) const;
    float PredictCorrectedPower(const int16_t* turn_current,
                                const float* angular_velocity,
                                uint8_t motor_num,
                                float common_scale) const;
    float GetControlPowerBudget() const;
    void UpdateDynamicScale();
    void UpdateModelCorrection(float truth_power, float model_power, float alpha);
    void UpdateRefereeModelCorrection();
    void SettleLocalRule(float observed_power);

    struct GuardSnapshot {
        bool enabled;
        bool local_trip;
        float power_budget;
        float voltage;
        float model_correction;
        float lqr_scale;
    };

    void PublishGuardSnapshot();
    GuardSnapshot ReadGuardSnapshot() const;

    KFPowerEstimator kf_estimator_;
    SeasonRuleManager rule_manager_;
    Config config_;

    bool enabled_;
    bool referee_online_;
    float max_watt_;
    float voltage_;
    float measured_power_;
    float external_buffer_joule_;

    float raw_model_power_;
    float requested_power_;
    float limited_power_estimate_;
    float power_prediction_;
    float power_rate_;
    float model_correction_;

    float current_scale_;
    float lqr_scale_;
    float hard_scale_;
    float predicted_buffer_;

    float local_bus_power_;
    float local_bus_voltage_;
    float local_bus_current_amp_;
    float local_bus_age_s_;
    bool local_bus_valid_;

    uint32_t referee_sample_sequence_;
    uint32_t last_fused_referee_sequence_;
    bool referee_sample_initialized_;
    float last_model_window_power_;
    bool model_window_initialized_;
    float model_window_sum_;
    uint16_t model_window_count_;

    float settle_power_sum_;
    uint16_t settle_sample_count_;
    uint16_t settle_sample_target_;
    uint32_t local_trip_remaining_steps_;

    GuardSnapshot guard_snapshots_[2];
    volatile uint8_t guard_snapshot_index_;

    static constexpr uint8_t CURRENT_SCALE_SEARCH_ITERATIONS = 16;
    static constexpr float MODEL_CORRECTION_MIN = 0.60f;
    static constexpr float MODEL_CORRECTION_MAX = 1.80f;
    static constexpr float MIN_CORRECTION_POWER_W = 5.0f;
    static constexpr float LOCAL_TRIP_SECONDS = 5.0f;
};

}  // namespace control
