/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
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

#include "MotorCanBase.h"
#include "can_bridge.h"
#include "connection_driver.h"
#include "pid.h"
#include "power_limit_new.h"
#include "power_limit_lqr.h"
#include "supercap.h"

#define MAX_WHEEL_NUM 8

namespace control {

    /**
     * @brief chassis models
     */
    typedef enum { CHASSIS_MECANUM_WHEEL, CHASSIS_OMNI_WHEEL, CHASSIS_OMNI_WHEEL2 } chassis_model_t;

    typedef struct {
        // Wheel_angle_a + Wheel_angle_b = 90
        int Center_point_angle;  // 中心点夹角，单位为度
        int Wheel_angle_a;       // 轮子的角度A，单位为度
        int Wheel_angle_b;       // 轮子的角度B，单位为度
        float chassis_radius;    // 轮子半径，单位为米
    } OMNI_Wheel_set_up;

    /**
     * @brief structure used when chassis instance is initialized
     */
    struct chassis_t {
        driver::MotorCANBase** motors;
        chassis_model_t model;
        float offset = 0;
        bool power_limit_on = false;
        bool has_super_capacitor = false;
        driver::SuperCap* super_capacitor = nullptr;
        OMNI_Wheel_set_up omni_wheel_setup_;
        
        bool use_lqr_power_limit = false;
        SeasonRuleManager::SeasonType season = SeasonRuleManager::SEASON_2026;
        bool blood_priority = true;
        float referee_power_safety_margin = 0.90f;
        float local_power_safety_margin = 0.90f;
    };

    /**
     * @brief motor configs for four wheel vehicles
     */
    struct FourWheel {
        enum { front_left, front_right, back_left, back_right, motor_num };
    };

    /**
     * @brief wrapper class for chassis
     */
    class Chassis : public driver::ConnectionDriver {
      public:
        /**
         * @brief constructor for chassis
         *
         * @param chassis structure that used to initialize chassis, refer to type
         * chassis_t
         */
        explicit Chassis(const chassis_t chassis);

        /**
         * @brief destructor for chassis
         */
        ~Chassis() override;

        /**
         * @brief 根据底盘整体速度，解算每个电机的速度，并储存起来
         *
         * @param x_speed chassis speed on x-direction
         * @param y_speed chassis speed on y-direction
         * @param turn_speed chassis clockwise turning speed
         */
        void SetSpeed(const float x_speed, const float y_speed = 0, const float turn_speed = 0);

        /**
         * @brief 使用外部采集的底盘数据，更新功率控制信息。
         * @note 这里仅记录信息，实际在回调函数中进行功率控制的计算。
         * @warning 已弃用，需要使用新的功率控制函数
         * @param enabled 功率控制开关
         * @param max_power total power limit, in [W]
         * @param current_power Current chassis power, in [W]
         * @param buffer_remain Current chassis power buffer, in [J]
         */
        void SetPower(bool enabled, float max_power, float current_power, float buffer_remain,
                      bool enable_supercap = false);

        /**
         * @brief 更新 2026 底盘功率控制输入。
         * @param enabled 功率控制开关
         * @param max_watt 裁判系统当前功率上限 Pl；无裁判时为本地测试上限
         * @param current_voltage 电池电压，单位 V
         * @param buffer_joule 当前缓冲能量 Z，2026 规则范围 0~60 J
         * @param measured_power 裁判系统实测底盘功率；不可用时传负数
         * @param referee_online 参数是否来自在线裁判系统
         */
        void UpdatePower(bool enabled, float max_watt, float current_voltage,
                         float buffer_joule, float measured_power,
                         bool referee_online);

        /** 兼容旧调用：按本地来源将百分比换算为 0~60 J。 */
        void UpdatePower(bool enabled, float max_watt, float current_voltage,
                         uint8_t buffer_percent);

        /**
         * @brief 在双板通信模式下，底盘使用自己采样的电压，因此需要额外暴露接口
         */
        void UpdatePowerVoltage(float voltage);

        /**
         * @brief 将解算得到的数据（每个电机的转速）传递给电机类，由电机类进行PID控制、CAN输出等
         * @note 检测底盘是否关闭，以及电机是否掉线（电机掉线则关闭底盘，需要手动重启）。
         */
        void Update();

        void Enable();

        void Disable();

        void SetMaxMotorSpeed(float max_speed);
        
        float GetPowerEstimate() const;
        
        float GetPowerPrediction() const;

        /** 限流后、经裁判实测修正的功率估算。 */
        float GetLimitedPowerEstimate() const;
        float GetMeasuredPower() const;
        float GetPowerLimit() const;
        float GetModelCorrection() const;
        bool IsRefereePowerSource() const;
        bool IsLocalPowerTripActive() const;
        
        float GetBuffer() const;
        
        bool IsPowerWarning() const;
        
        SeasonRuleManager::WarnType GetWarnType() const;
        
        float GetSpeedScale() const;

        /** Kinematic pre-PID scaling applied by SetSpeed(). */
        float GetKinematicScale() const;

        /**
         * @brief 获取实际应用到四轮电流上的公共缩放系数。
         */
        float GetCurrentScale() const;
        
        void SetBloodPriority(bool blood_priority);
        
        void SetSeason(SeasonRuleManager::SeasonType season);

      public:
        // 在所有电机的PID计算完成、准备发送CAN前，调用此函数直接设置电机输出，以进行功率限制
        static void ApplyPowerLimitWrapper(void* args);

      private:
        void ApplyPowerLimit();

      public:
        void CanBridgeSetTxId(uint8_t tx_id);

        static void CanBridgeUpdateEventXYWrapper(communication::can_bridge_ext_id_t ext_id,
                                                  communication::can_bridge_data_t data,
                                                  void* args);

        static void CanBridgeUpdateEventTurnWrapper(communication::can_bridge_ext_id_t ext_id,
                                                    communication::can_bridge_data_t data,
                                                    void* args);

        static void CanBridgeUpdateEventPowerLimitWrapper(communication::can_bridge_ext_id_t ext_id,
                                                          communication::can_bridge_data_t data,
                                                          void* args);

        static void CanBridgeUpdateEventCurrentPowerWrapper(
            communication::can_bridge_ext_id_t ext_id, communication::can_bridge_data_t data,
            void* args);

        void CanBridgeUpdateEventXY(communication::can_bridge_ext_id_t ext_id,
                                    communication::can_bridge_data_t data);

        void CanBridgeUpdateEventTurn(communication::can_bridge_ext_id_t ext_id,
                                      communication::can_bridge_data_t data);

        void CanBridgeUpdateEventPowerLimit(communication::can_bridge_ext_id_t ext_id,
                                            communication::can_bridge_data_t data);

        void CanBridgeUpdateEventCurrentPower(communication::can_bridge_ext_id_t ext_id,
                                              communication::can_bridge_data_t data);

        OMNI_Wheel_set_up omni_wheel_setup_{
            .Center_point_angle = 45,
            .Wheel_angle_a = 90 - omni_wheel_setup_.Center_point_angle,
            .Wheel_angle_b = omni_wheel_setup_.Center_point_angle,
            .chassis_radius = 0.085f,
        };

        volatile float can_bridge_vx_ = 0;
        volatile float can_bridge_vy_ = 0;
        volatile float can_bridge_vt_ = 0;
        bool chassis_enable_ = true;

      private:
        // acquired from user
        driver::MotorCANBase** motors_ = nullptr;
        chassis_model_t model_;

        // pids and current speeds for each motor on the chassis
        float* speeds_ = nullptr;

        uint8_t wheel_num_ = 0;

        struct {
            bool enabled = false;
            bool referee_online = false;
            float buffer_joule = 60.0f;
            float measured_power = -1.0f;
            float max_watt = 0.0f;
            float voltage = 0.0f;

            NewPowerLimit* limiter = nullptr;
        } power_limit_;
        
        LQRPowerLimit* lqr_power_limit_ = nullptr;
        bool use_lqr_power_limit_ = false;

        float chassis_offset_;

        uint8_t can_bridge_tx_id_ = 0x00;

        bool has_super_capacitor_ = false;
        bool super_capacitor_enable_ = false;
        driver::SuperCap* super_capacitor_ = nullptr;

        float max_motor_speed_ = 2 * PI * 10;
        float kinematic_scale_ = 1.0f;

      public:
        // Record for debugging
        float chassis_vx = 0;
        float chassis_vy = 0;
        float chassis_vt = 0;
    };

    class ChassisCanBridgeSender {
      public:
        ChassisCanBridgeSender(communication::CanBridge* can_bridge, uint8_t rx_id);
        void SetChassisRegId(uint8_t xy_reg_id, uint8_t turn_on_reg_id, uint8_t power_limit_reg_id,
                             uint8_t current_power_reg_id);
        void Enable();
        void Disable();
        /**
         * @brief set the speed for chassis motors
         *
         * @param x_speed chassis speed on x-direction
         * @param y_speed chassis speed on y-direction
         * @param turn_speed chassis clockwise turning speed
         */
        void SetSpeed(const float x_speed, const float y_speed = 0, const float turn_speed = 0);

        /**
         * @brief set the power limit for chassis motors
         * @warning deprecated, use UpdatePower instead
         * @param power_limit_on whether to enable power limit
         * @param power_limit total power limit, in [W]
         * @param chassis_power Current chassis power, in [W]
         * @param chassis_power_buffer Current chassis power buffer, in [J]
         */
        void SetPower(bool power_limit_on, float power_limit, float chassis_power,
                      float chassis_power_buffer, bool enable_supercap = false,
                      bool force_update = false);

        // Same as Chassis::UpdatePower. Sends limit/voltage and feedback on
        // the existing 0x72/0x73 bridge registers.
        void UpdatePower(bool enabled, uint8_t max_watt, float current_voltage,
                         float buffer_joule, float measured_power,
                         bool referee_online);

        /** Compatibility overload for local percent-based callers. */
        void UpdatePower(bool enabled, uint8_t max_watt, float current_voltage,
                         uint8_t buffer_percent);

      private:
        communication::CanBridge* can_bridge_;
        bool chassis_enable_ = true;
        bool chassis_power_limit_on_ = false;
        bool chassis_super_capacitor_enable_ = false;
        float chassis_power_limit_ = 120;
        uint8_t device_rx_id_ = 0x00;
        uint8_t chassis_xy_reg_id_ = 0x00;
        uint8_t chassis_turn_on_reg_id_ = 0x00;
        uint8_t chassis_power_limit_reg_id_ = 0x00;
        uint8_t chassis_current_power_reg_id_ = 0x00;
        communication::can_bridge_ext_id_t rx_id_;
        communication::can_bridge_data_t data_;

      public:
        // Record for debugging
        float chassis_vx = 0;
        float chassis_vy = 0;
        float chassis_vt = 0;
    };

}  // namespace control
