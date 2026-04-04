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

    /**
     * RoboMaster season-level buffer accounting.
     *
     * The robot's current power limit is supplied by the referee system.  It
     * must not be encoded here because the 2026 infantry limit depends on the
     * selected chassis type, level and temporary buffs.
     */
    class SeasonRuleManager {
    public:
        enum SeasonType {
            SEASON_2025 = 0,
            SEASON_2026 = 1
        };

        enum WarnType {
            WARN_NONE = 0,
            WARN_BUFFER = 1,
            WARN_PREDICT = 2,
            WARN_IMMEDIATE = 3
        };

        explicit SeasonRuleManager(SeasonType season = SEASON_2026);

        /** Apply Z = clamp(Z - (Pr - Pl) * dt, 0, Q). */
        void CalcBufferConsume(float measured_power, float power_limit, float dt);

        /** Synchronise to the referee-system buffer, which is authoritative. */
        void SyncBuffer(float buffer_joule);

        void JudgeWarn(float measured_power, float predicted_power, float power_limit);

        /** Legacy overload; uses the most recently supplied dynamic limit. */
        void JudgeWarn(float measured_power, float predicted_power,
                       bool blood_priority);

        float GetSpeedScale() const;
        bool IsWarning() const;
        WarnType GetWarnType() const;
        float GetBuffer() const;
        /** Legacy API: returns the most recently supplied dynamic referee/local limit. */
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

    class LQRPowerLimit {
    public:
        struct Config {
            SeasonRuleManager::SeasonType season;
            bool blood_priority;  // Retained for source compatibility; referee limit is authoritative.
            float dt;
            float battery_voltage;
            float referee_safety_margin;
            float local_safety_margin;

            Config()
                : season(SeasonRuleManager::SEASON_2026),
                  blood_priority(true),
                  dt(0.001f),
                  battery_voltage(24.0f),
                  referee_safety_margin(0.90f),
                  local_safety_margin(0.90f) {}
        };

        LQRPowerLimit();
        explicit LQRPowerLimit(const Config& config);
        ~LQRPowerLimit();

        /**
         * Full 2026 power state.
         *
         * @param buffer_joule Remaining official/local buffer in joules (0..60 for 2026).
         * @param measured_power Referee measured chassis power, or a negative value when unavailable.
         * @param referee_online True only when the values are valid referee-system data.
         */
        void UpdatePowerState(bool enabled, float max_watt, float voltage,
                              float buffer_joule, float measured_power,
                              bool referee_online);

        /** Backward-compatible local-source overload. */
        void UpdatePowerState(bool enabled, float max_watt, float voltage,
                              uint8_t buffer_percent);

        void UpdateMotorData(const float* omega, const int16_t* current, uint8_t motor_num);
        void ApplyLimit(int16_t* turn_current, const float* angular_velocity, uint8_t motor_num);

        float GetSpeedScale() const;
        float GetPowerEstimate() const;          // request, before limiting
        float GetPowerPrediction() const;        // request prediction, before limiting
        float GetLimitedPowerEstimate() const;   // corrected estimate after limiting
        float GetMeasuredPower() const;
        float GetPowerLimit() const;
        float GetModelCorrection() const;
        float GetCurrentScale() const;
        float GetBuffer() const;
        bool IsRefereeSource() const;
        bool IsLocalTripActive() const;
        bool IsWarning() const;
        SeasonRuleManager::WarnType GetWarnType() const;

        void Reset();
        void SetBloodPriority(bool blood_priority);
        void SetSeason(SeasonRuleManager::SeasonType season);

    private:
        float CalcRequestedPower(const int16_t* current, uint8_t motor_num) const;
        float PowerModel(float angular_velocity, float turn_current) const;
        float PredictTotalEquivalentCurrent(const int16_t* turn_current,
                                            const float* angular_velocity,
                                            uint8_t motor_num,
                                            float common_scale) const;
        float PredictCorrectedPower(const int16_t* turn_current,
                                    const float* angular_velocity,
                                    uint8_t motor_num,
                                    float common_scale) const;
        float GetControlPowerBudget() const;
        void UpdateModelCorrection();
        void SettleLocalRule(float limited_power);

        KFPowerEstimator kf_estimator_;
        SeasonRuleManager rule_manager_;
        Config config_;

        bool enabled_;
        bool referee_online_;
        float max_watt_;
        float voltage_;
        float measured_power_;
        float external_buffer_joule_;
        float requested_power_;
        float limited_power_estimate_;
        float model_correction_;
        float current_scale_;
        float last_correction_sample_power_;
        bool correction_sample_initialized_;

        float settle_power_sum_;
        uint16_t settle_sample_count_;
        uint32_t local_trip_remaining_steps_;

        static constexpr float CURRENT_SCALE_RELEASE_STEP = 0.01f;
        static constexpr uint8_t CURRENT_SCALE_SEARCH_ITERATIONS = 16;
        static constexpr float MODEL_CORRECTION_ALPHA = 0.05f;
        static constexpr float MODEL_CORRECTION_MIN = 0.60f;
        static constexpr float MODEL_CORRECTION_MAX = 1.80f;
        static constexpr float MIN_CORRECTION_POWER_W = 5.0f;
        static constexpr float LOCAL_TRIP_SECONDS = 5.0f;

        // M3508/C620 equivalent input-current model.  The result is mA.
        static constexpr float K1 = 0.000931f;
        static constexpr float K2 = 0.000455f;
        static constexpr float K3 = 0.000006f;
        static constexpr float K4 = 39.151850f;

        static constexpr uint16_t SETTLE_BUFFER_SIZE = 100;
    };

}  // namespace control
