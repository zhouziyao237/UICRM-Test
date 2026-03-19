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
        
        void CalcBufferConsume(float P_settle, float P_limit, float dt);
        
        void JudgeWarn(float P_meas, float P_pred, bool blood_priority);
        
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
        void CalcSpeedScale(float P_meas, float P_limit);
        
        SeasonType season_;
        
        float P_limit_blood_;
        float P_limit_power_;
        float Z_max_;
        float warn_threshold_;
        float settle_period_;
        float speed_scale_max_;
        
        float Z_;
        bool warn_flag_;
        WarnType warn_type_;
        float speed_scale_;
    };

    class LQRPowerLimit {
    public:
        struct Config {
            SeasonRuleManager::SeasonType season;
            bool blood_priority;
            float dt;
            float battery_voltage;
            
            Config() : season(SeasonRuleManager::SEASON_2026), 
                       blood_priority(true), 
                       dt(0.001f), 
                       battery_voltage(24.0f) {}
        };

        LQRPowerLimit();
        explicit LQRPowerLimit(const Config& config);
        
        ~LQRPowerLimit();
        
        void UpdatePowerState(bool enabled, float max_watt, float voltage, uint8_t buffer_percent);
        
        void UpdateMotorData(const float* omega, const int16_t* current, uint8_t motor_num);
        
        void ApplyLimit(int16_t* turn_current, const float* angular_velocity, uint8_t motor_num);
        
        float GetSpeedScale() const;
        
        float GetPowerEstimate() const;
        
        float GetPowerPrediction() const;
        
        float GetBuffer() const;
        
        bool IsWarning() const;
        
        SeasonRuleManager::WarnType GetWarnType() const;
        
        void Reset();
        
        void SetBloodPriority(bool blood_priority);
        
        void SetSeason(SeasonRuleManager::SeasonType season);

    private:
        float CalcInstantPower(const int16_t* current, uint8_t motor_num);
        
        float PowerModel(float angular_velocity, int16_t turn_current);
        
        int16_t ReversePowerModel(float angular_velocity, int16_t turn_current, float target_current);
        
        void UpdateSettleBuffer(float power);
        
        KFPowerEstimator kf_estimator_;
        SeasonRuleManager rule_manager_;
        
        Config config_;
        
        bool enabled_;
        float max_watt_;
        float voltage_;
        float current_power_;
        uint8_t buffer_percent_;
        
        static constexpr float K1 = 0.000931f;
        static constexpr float K2 = 0.000455f;
        static constexpr float K3 = 0.000006f;
        static constexpr float K4 = 39.151850f;
        
        static constexpr uint16_t SETTLE_BUFFER_SIZE = 100;
        float power_buffer_[SETTLE_BUFFER_SIZE];
        uint16_t power_buffer_idx_;
        float power_sum_;
        uint16_t power_count_;
    };

}  // namespace control
