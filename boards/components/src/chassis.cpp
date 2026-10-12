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

#include "chassis.h"

#include <cmath>

#include "bsp_error_handler.h"
#include "bsp_os.h"

#define M3508_POWER_MODEL {0.000931, 0.000455, 0.000006, 39.151850}

namespace control {

    Chassis::Chassis(const chassis_t chassis) {
        // acquired from user
        model_ = chassis.model;
        if (model_ == CHASSIS_OMNI_WHEEL2) {
            omni_wheel_setup_ = chassis.omni_wheel_setup_;
        }

        // data initialization using acquired model
        switch (chassis.model) {
                // 麦克纳姆轮
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                // 新建电机关联
                motors_ = new driver::MotorCANBase*[FourWheel::motor_num];
                motors_[FourWheel::front_left] = chassis.motors[FourWheel::front_left];
                motors_[FourWheel::front_right] = chassis.motors[FourWheel::front_right];
                motors_[FourWheel::back_left] = chassis.motors[FourWheel::back_left];
                motors_[FourWheel::back_right] = chassis.motors[FourWheel::back_right];

                // 速度初始化
                speeds_ = new float[FourWheel::motor_num];
                for (int i = 0; i < FourWheel::motor_num; ++i)
                    speeds_[i] = 0;

                // 轮子数
                wheel_num_ = FourWheel::motor_num;
                break;
            }
            case CHASSIS_OMNI_WHEEL2: {
                // 新建电机关联
                motors_ = new driver::MotorCANBase*[FourWheel::motor_num];
                motors_[FourWheel::front_left] = chassis.motors[FourWheel::front_left];
                motors_[FourWheel::front_right] = chassis.motors[FourWheel::front_right];
                motors_[FourWheel::back_left] = chassis.motors[FourWheel::back_left];
                motors_[FourWheel::back_right] = chassis.motors[FourWheel::back_right];

                // 功率限制系统 todo: 老功率限制代码
                // power_limit_ = new PowerLimit(FourWheel::motor_num);

                // 速度初始化
                speeds_ = new float[FourWheel::motor_num];
                for (int i = 0; i < FourWheel::motor_num; ++i)
                    speeds_[i] = 0;

                // 轮子数
                wheel_num_ = FourWheel::motor_num;
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
        chassis_offset_ = chassis.offset;

        // 功率限制系统
        NewPowerLimit::power_param_t power_model[4] = {M3508_POWER_MODEL, M3508_POWER_MODEL,
                                                       M3508_POWER_MODEL, M3508_POWER_MODEL};
        power_limit_.enabled = chassis.power_limit_on;
        power_limit_.limiter = new NewPowerLimit(power_model);
        driver::MotorCANBase::RegisterPreOutputCallback(ApplyPowerLimitWrapper, this);

        // LQR功率限制系统
        use_lqr_power_limit_ = chassis.use_lqr_power_limit;
        if (use_lqr_power_limit_) {
            LQRPowerLimit::Config lqr_config = chassis.use_custom_power_control_config
                                                    ? chassis.power_control_config
                                                    : LQRPowerLimit::Config();
            // The legacy fields remain authoritative unless the caller opts in
            // to a complete per-vehicle profile.
            if (!chassis.use_custom_power_control_config) {
                lqr_config.season = chassis.season;
                lqr_config.blood_priority = chassis.blood_priority;
                lqr_config.referee_safety_margin =
                    chassis.referee_power_safety_margin;
                lqr_config.local_safety_margin =
                    chassis.local_power_safety_margin;
            }
            lqr_power_limit_ = new LQRPowerLimit(lqr_config);
        }

        // 底盘是否有超级电容
        if (chassis.has_super_capacitor) {
            has_super_capacitor_ = true;
            super_capacitor_ = chassis.super_capacitor;
        }
    }

    Chassis::~Chassis() {
        driver::MotorCANBase::RegisterPreOutputCallback([](void* args) { UNUSED(args); }, nullptr);
        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                motors_[FourWheel::front_left] = nullptr;
                motors_[FourWheel::front_right] = nullptr;
                motors_[FourWheel::back_left] = nullptr;
                motors_[FourWheel::back_right] = nullptr;
                delete[] motors_;
                motors_ = nullptr;

                delete[] speeds_;
                speeds_ = nullptr;
                break;
            }
            case CHASSIS_OMNI_WHEEL2: {
                motors_[FourWheel::front_left] = nullptr;
                motors_[FourWheel::front_right] = nullptr;
                motors_[FourWheel::back_left] = nullptr;
                motors_[FourWheel::back_right] = nullptr;
                delete[] motors_;
                motors_ = nullptr;

                delete[] speeds_;
                speeds_ = nullptr;
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

    void Chassis::SetSpeed(const float x_speed, const float y_speed, const float turn_speed) {
        Heartbeat();
        chassis_vx = x_speed;
        chassis_vy = y_speed;
        chassis_vt = turn_speed;

        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL: {
                float move_sum = fabs(x_speed) + fabs(y_speed) + fabs(turn_speed);
                float scale = move_sum > max_motor_speed_ ? max_motor_speed_ / move_sum : 1.0f;
                kinematic_scale_ = scale;

                speeds_[FourWheel::front_left] =
                    scale * (y_speed + x_speed + turn_speed * (1 - chassis_offset_));  // 2
                speeds_[FourWheel::back_left] =
                    scale * (y_speed - x_speed + turn_speed * (1 + chassis_offset_));  // 3
                speeds_[FourWheel::front_right] =
                    -scale * (y_speed - x_speed - turn_speed * (1 - chassis_offset_));  // 1
                speeds_[FourWheel::back_right] =
                    -scale * (y_speed + x_speed - turn_speed * (1 + chassis_offset_));  // 4
                break;
            }
            case CHASSIS_OMNI_WHEEL2: {
                const float angle_a_rad = omni_wheel_setup_.Wheel_angle_a * PI / 180.0f;
                const float angle_b_rad = omni_wheel_setup_.Wheel_angle_b * PI / 180.0f;

                float scale = 1.0f;
                float move_sum = fabsf(x_speed) + fabsf(y_speed) + fabsf(turn_speed);
                scale = move_sum > max_motor_speed_ ? max_motor_speed_ / move_sum : 1.0f;
                kinematic_scale_ = scale;

                speeds_[FourWheel::front_left] =
                    scale * (-x_speed * cosf(angle_a_rad) + y_speed * sinf(angle_b_rad) +
                             turn_speed);  // 1
                speeds_[FourWheel::front_right] =
                    scale * (x_speed * cosf(angle_a_rad) + y_speed * sinf(angle_b_rad) +
                             turn_speed);  // 2
                speeds_[FourWheel::back_left] =
                    scale * (-x_speed * cosf(angle_a_rad) - y_speed * sinf(angle_b_rad) +
                             turn_speed);  // 3
                speeds_[FourWheel::back_right] =
                    scale * (x_speed * cosf(angle_a_rad) - y_speed * sinf(angle_b_rad) +
                             turn_speed);  // 4
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

    void Chassis::SetPower(bool enabled, float max_power, float current_power, float buffer_remain,
                           bool enable_supercap) {
        power_limit_.enabled = enabled;

        UNUSED(max_power);
        UNUSED(current_power);
        UNUSED(buffer_remain);
        UNUSED(enable_supercap);

        return;
    }

    void Chassis::UpdatePower(bool enabled, float max_watt,
                              float current_voltage, float buffer_joule,
                              float measured_power, bool referee_online) {
        power_limit_.enabled = enabled;
        power_limit_.referee_online = referee_online;
        power_limit_.buffer_joule =
            std::isfinite(buffer_joule)
                ? (buffer_joule < 0.0f
                       ? 0.0f
                       : (buffer_joule > 60.0f ? 60.0f : buffer_joule))
                : 0.0f;
        power_limit_.measured_power =
            std::isfinite(measured_power) && measured_power >= 0.0f
                ? measured_power
                : -1.0f;
        if (std::isfinite(current_voltage) && current_voltage > 1.0f) {
            power_limit_.voltage = current_voltage;
        }
        power_limit_.max_watt =
            std::isfinite(max_watt) && max_watt > 0.0f ? max_watt : 0.0f;
    }

    void Chassis::UpdatePower(bool enabled, float max_watt,
                              float current_voltage,
                              uint8_t buffer_percent) {
        const float buffer_joule =
            60.0f * static_cast<float>(buffer_percent > 100 ? 100
                                                            : buffer_percent) /
            100.0f;
        UpdatePower(enabled, max_watt, current_voltage, buffer_joule, -1.0f,
                    false);
    }

    void Chassis::UpdatePowerVoltage(float voltage) {
        if (std::isfinite(voltage) && voltage > 1.0f) {
            power_limit_.voltage = voltage;
        }
    }

    void Chassis::UpdateLocalPowerMeasurement(float voltage, float current_amp,
                                              bool valid) {
        if (std::isfinite(voltage) && voltage > 1.0f) {
            power_limit_.voltage = voltage;
        }
        if (use_lqr_power_limit_ && lqr_power_limit_) {
            lqr_power_limit_->UpdateLocalBusMeasurement(voltage, current_amp, valid);
        }
    }

    void Chassis::InvalidateLocalPowerMeasurement() {
        if (use_lqr_power_limit_ && lqr_power_limit_) {
            lqr_power_limit_->InvalidateLocalBusMeasurement();
        }
    }

    void Chassis::PowerControlUpdate(float local_voltage) {
        if (!use_lqr_power_limit_ || lqr_power_limit_ == nullptr) return;

        const float voltage =
            std::isfinite(local_voltage) && local_voltage > 1.0f
                ? local_voltage
                : power_limit_.voltage;

        int16_t requested_current[FourWheel::motor_num];
        float angular_velocity[FourWheel::motor_num];
        for (uint8_t i = 0; i < FourWheel::motor_num; ++i) {
            // Read-only snapshot of the most recently calculated motor command.
            // Final safety does not depend on this snapshot: ApplyPowerLimit()
            // re-reads the live command immediately before CAN TX.
            requested_current[i] = motors_[i]->GetOutput();
            angular_velocity[i] = motors_[i]->GetOmega();
        }

        lqr_power_limit_->UpdatePowerState(
            power_limit_.enabled, power_limit_.max_watt, voltage,
            power_limit_.buffer_joule, power_limit_.measured_power,
            power_limit_.referee_online, power_limit_.referee_sample_sequence);
        lqr_power_limit_->UpdateMotorData(angular_velocity, requested_current,
                                          FourWheel::motor_num);
    }

    void Chassis::Update() {
        // 如果有电机掉线，则关闭整个底盘
        bool need_shutdown = !IsOnline();
        for (int i = 0; i < wheel_num_; i++) {
            // 如果某个电机掉线，底盘将被禁用
            // 什么几把逻辑，连报错都没有就直接禁用是吧
            if (!motors_[i]->IsOnline()) {
                need_shutdown = true;
                print("Checked motor offline, Kill Chassis!\r\n");
                break;
            }
        }

        // 如果此标志置否，则底盘电机将被禁用
        if (need_shutdown) {
            Disable();
        }

        // 根据底盘的开关，控制四个电机开关
        if (!chassis_enable_) {
            for (int i = 0; i < wheel_num_; i++) {
                motors_[i]->Disable();
            }
            return;
        }
        for (int i = 0; i < wheel_num_; i++) {
            if (!motors_[i]->IsEnable())
                motors_[i]->Enable();
        }

        // 将解算得到的数据（每个电机的转速）传递给电机类，由电机类进行PID控制、CAN输出等
        switch (model_) {
            case CHASSIS_MECANUM_WHEEL:
            case CHASSIS_OMNI_WHEEL:
            case CHASSIS_OMNI_WHEEL2: {
                motors_[FourWheel::front_left]->SetTarget(speeds_[FourWheel::front_left]);
                motors_[FourWheel::front_right]->SetTarget(speeds_[FourWheel::front_right]);
                motors_[FourWheel::back_left]->SetTarget(speeds_[FourWheel::back_left]);
                motors_[FourWheel::back_right]->SetTarget(speeds_[FourWheel::back_right]);
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

    void Chassis::CanBridgeUpdateEventXYWrapper(communication::can_bridge_ext_id_t ext_id,
                                                communication::can_bridge_data_t data, void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventXY(ext_id, data);
    }

    void Chassis::CanBridgeUpdateEventTurnWrapper(communication::can_bridge_ext_id_t ext_id,
                                                  communication::can_bridge_data_t data,
                                                  void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventTurn(ext_id, data);
    }

    void Chassis::CanBridgeUpdateEventPowerLimitWrapper(communication::can_bridge_ext_id_t ext_id,
                                                        communication::can_bridge_data_t data,
                                                        void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventPowerLimit(ext_id, data);
    }

    void Chassis::CanBridgeUpdateEventCurrentPowerWrapper(communication::can_bridge_ext_id_t ext_id,
                                                          communication::can_bridge_data_t data,
                                                          void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->CanBridgeUpdateEventCurrentPower(ext_id, data);
    }

    void Chassis::CanBridgeUpdateEventXY(communication::can_bridge_ext_id_t ext_id,
                                         communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00 && can_bridge_tx_id_ != ext_id.data.tx_id) {
            return;
        }

        const uint8_t sequence = ext_id.data.res & 0x07U;
        if (can_bridge_turn_valid_ && can_bridge_turn_seq_ != sequence) {
            // Do not allow an older turn frame to combine with a newer XY frame.
            can_bridge_turn_valid_ = false;
        }
        can_bridge_pending_vx_ = data.data_two_float.data[0];
        can_bridge_pending_vy_ = data.data_two_float.data[1];
        can_bridge_xy_seq_ = sequence;
        can_bridge_xy_valid_ = true;
        TryCommitCanBridgeMotion();
    }

    void Chassis::CanBridgeUpdateEventTurn(communication::can_bridge_ext_id_t ext_id,
                                           communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00 && can_bridge_tx_id_ != ext_id.data.tx_id) {
            return;
        }

        const float is_enable = data.data_two_float.data[0];
        if (is_enable < 0.1f) {
            // Disable is fail-safe and therefore commits immediately; it never waits
            // for the matching XY frame.
            can_bridge_xy_valid_ = false;
            can_bridge_turn_valid_ = false;
            can_bridge_vx_ = 0.0f;
            can_bridge_vy_ = 0.0f;
            can_bridge_vt_ = 0.0f;
            if (chassis_enable_) {
                Disable();
            }
            SetSpeed(0.0f, 0.0f, 0.0f);
            return;
        }

        const uint8_t sequence = ext_id.data.res & 0x07U;
        if (can_bridge_xy_valid_ && can_bridge_xy_seq_ != sequence) {
            // Do not allow an older XY frame to combine with a newer turn frame.
            can_bridge_xy_valid_ = false;
        }
        can_bridge_pending_vt_ = data.data_two_float.data[1];
        can_bridge_turn_seq_ = sequence;
        can_bridge_turn_valid_ = true;
        TryCommitCanBridgeMotion();
    }

    void Chassis::TryCommitCanBridgeMotion() {
        if (!can_bridge_xy_valid_ || !can_bridge_turn_valid_ ||
            can_bridge_xy_seq_ != can_bridge_turn_seq_) {
            return;
        }

        can_bridge_vx_ = can_bridge_pending_vx_;
        can_bridge_vy_ = can_bridge_pending_vy_;
        can_bridge_vt_ = can_bridge_pending_vt_;
        can_bridge_xy_valid_ = false;
        can_bridge_turn_valid_ = false;

        if (!chassis_enable_) {
            Enable();
        }
        SetSpeed(can_bridge_vx_, can_bridge_vy_, can_bridge_vt_);
    }
    void Chassis::CanBridgeUpdateEventPowerLimit(communication::can_bridge_ext_id_t ext_id,
                                                 communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00) {
            if (can_bridge_tx_id_ != ext_id.data.tx_id) {
                return;
            }
        }

        const bool enabled = data.data_eight_uint8.data[0] != 0U;
        const uint8_t max_watt = data.data_eight_uint8.data[1];
        const float buffer_joule =
            static_cast<float>(data.data_eight_uint8.data[2]);
        const bool referee_online = data.data_eight_uint8.data[3] != 0U;
        const float voltage = data.data_two_float.data[1];
        ++power_limit_.bridge_update_sequence;
        UpdatePower(enabled, max_watt, voltage, buffer_joule,
                    power_limit_.measured_power, referee_online);
    }
    void Chassis::CanBridgeUpdateEventCurrentPower(communication::can_bridge_ext_id_t ext_id,
                                                   communication::can_bridge_data_t data) {
        if (can_bridge_tx_id_ != 0x00) {
            if (can_bridge_tx_id_ != ext_id.data.tx_id) {
                return;
            }
        }
        const float measured_power = data.data_two_float.data[0];
        const float buffer_joule = data.data_two_float.data[1];
        // 0x73 is emitted at the 100 ms referee-power update cadence.  Sequence
        // by arrival, not by value, so equal consecutive Pr samples are still
        // fused exactly once by the Kalman estimator.
        ++power_limit_.referee_sample_sequence;
        ++power_limit_.bridge_update_sequence;
        UpdatePower(power_limit_.enabled, power_limit_.max_watt,
                    power_limit_.voltage, buffer_joule, measured_power,
                    power_limit_.referee_online);
    }
    void Chassis::CanBridgeSetTxId(uint8_t tx_id) {
        can_bridge_tx_id_ = tx_id;
    }
    void Chassis::ApplyPowerLimitWrapper(void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->ApplyPowerLimit();
    }
    void Chassis::ApplyPowerLimit() {
        if (!power_limit_.enabled) {
            return;
        }
        
        if (use_lqr_power_limit_ && lqr_power_limit_) {
            int16_t turn_current[FourWheel::motor_num];
            float angular_velocity[FourWheel::motor_num];
            
            for (uint8_t i = 0; i < FourWheel::motor_num; ++i) {
                turn_current[i] = motors_[i]->GetOutput();
                angular_velocity[i] = motors_[i]->GetOmega();
            }
            
            // The PowerManager thread owns estimation and dynamic LQR updates.
            // This callback is intentionally only the synchronous final hard
            // guard immediately before MotorCAN transmits the command.
            lqr_power_limit_->ApplyLimit(turn_current, angular_velocity,
                                         FourWheel::motor_num);
            
            for (uint8_t i = 0; i < FourWheel::motor_num; ++i) {
                motors_[i]->SetOutput(turn_current[i]);
            }
        } else {
            // 电流(A) = 功率(W) / 电压(V)
            float max_current = power_limit_.max_watt / power_limit_.voltage;

            // Legacy limiter path is strict: never invent a 1.5x buffer multiplier.
            max_current *= 0.90f;

            // 获取数据
            int16_t turn_current[FourWheel::motor_num];
            float angular_velocity[FourWheel::motor_num];
            for (uint8_t i = 0; i < FourWheel::motor_num; ++i) {
                turn_current[i] = motors_[i]->GetOutput();
                angular_velocity[i] = motors_[i]->GetOmega();
            }

            // 在转矩电流之上应用功率限制
            power_limit_.limiter->LimitPower(turn_current, angular_velocity,
                                             (int16_t)(max_current * 1000));

            // 应用限制后的转矩电流
            for (uint8_t i = 0; i < FourWheel::motor_num; ++i)
                motors_[i]->SetOutput((int16_t)turn_current[i]);
        }
    }
    void Chassis::Enable() {
        chassis_enable_ = true;
        if (has_super_capacitor_) {
            super_capacitor_->Enable();
        }
    }
    void Chassis::Disable() {
        chassis_enable_ = false;
        if (has_super_capacitor_) {
            super_capacitor_->Disable();
        }
    }

    void Chassis::SetMaxMotorSpeed(float max_speed) {
        max_motor_speed_ = max_speed;
    }
    
    float Chassis::GetPowerEstimate() const {
        if (use_lqr_power_limit_ && lqr_power_limit_) {
            return lqr_power_limit_->GetPowerEstimate();
        }
        return 0.0f;
    }
    
    float Chassis::GetPowerPrediction() const {
        if (use_lqr_power_limit_ && lqr_power_limit_) {
            return lqr_power_limit_->GetPowerPrediction();
        }
        return 0.0f;
    }

    float Chassis::GetPowerRate() const {
        return use_lqr_power_limit_ && lqr_power_limit_
                   ? lqr_power_limit_->GetPowerRate()
                   : 0.0f;
    }

    float Chassis::GetRequestedModelPower() const {
        return use_lqr_power_limit_ && lqr_power_limit_
                   ? lqr_power_limit_->GetRequestedModelPower()
                   : 0.0f;
    }

    float Chassis::GetLimitedPowerEstimate() const {
        return use_lqr_power_limit_ && lqr_power_limit_
                   ? lqr_power_limit_->GetLimitedPowerEstimate()
                   : 0.0f;
    }

    float Chassis::GetMeasuredPower() const {
        return use_lqr_power_limit_ && lqr_power_limit_
                   ? lqr_power_limit_->GetMeasuredPower()
                   : -1.0f;
    }

    float Chassis::GetPowerLimit() const {
        return use_lqr_power_limit_ && lqr_power_limit_
                   ? lqr_power_limit_->GetPowerLimit()
                   : power_limit_.max_watt;
    }

    float Chassis::GetModelCorrection() const {
        return use_lqr_power_limit_ && lqr_power_limit_
                   ? lqr_power_limit_->GetModelCorrection()
                   : 1.0f;
    }

    float Chassis::GetLocalBusPower() const {
        return use_lqr_power_limit_ && lqr_power_limit_
                   ? lqr_power_limit_->GetLocalBusPower()
                   : 0.0f;
    }

    bool Chassis::IsLocalBusPowerValid() const {
        return use_lqr_power_limit_ && lqr_power_limit_ &&
               lqr_power_limit_->IsLocalBusMeasurementValid();
    }

    float Chassis::GetControlPowerBudget() const {
        return use_lqr_power_limit_ && lqr_power_limit_
                   ? lqr_power_limit_->GetControlPowerBudgetPublic()
                   : power_limit_.max_watt;
    }

    float Chassis::GetLQRScale() const {
        return use_lqr_power_limit_ && lqr_power_limit_
                   ? lqr_power_limit_->GetLQRScale()
                   : 1.0f;
    }

    float Chassis::GetHardScale() const {
        return use_lqr_power_limit_ && lqr_power_limit_
                   ? lqr_power_limit_->GetHardScale()
                   : 1.0f;
    }

    float Chassis::GetPredictedBuffer() const {
        return use_lqr_power_limit_ && lqr_power_limit_
                   ? lqr_power_limit_->GetPredictedBuffer()
                   : 0.0f;
    }

    bool Chassis::IsRefereePowerSource() const {
        return use_lqr_power_limit_ && lqr_power_limit_ &&
               lqr_power_limit_->IsRefereeSource();
    }
    uint32_t Chassis::GetPowerBridgeSequence() const {
        return power_limit_.bridge_update_sequence;
    }

    bool Chassis::IsLocalPowerTripActive() const {
        return use_lqr_power_limit_ && lqr_power_limit_ &&
               lqr_power_limit_->IsLocalTripActive();
    }
    
    float Chassis::GetBuffer() const {
        if (use_lqr_power_limit_ && lqr_power_limit_) {
            return lqr_power_limit_->GetBuffer();
        }
        return 0.0f;
    }
    
    bool Chassis::IsPowerWarning() const {
        if (use_lqr_power_limit_ && lqr_power_limit_) {
            return lqr_power_limit_->IsWarning();
        }
        return false;
    }
    
    SeasonRuleManager::WarnType Chassis::GetWarnType() const {
        if (use_lqr_power_limit_ && lqr_power_limit_) {
            return lqr_power_limit_->GetWarnType();
        }
        return SeasonRuleManager::WARN_NONE;
    }
    
    float Chassis::GetSpeedScale() const {
        if (use_lqr_power_limit_ && lqr_power_limit_) {
            return lqr_power_limit_->GetSpeedScale();
        }
        return 1.0f;
    }

    float Chassis::GetKinematicScale() const {
        return kinematic_scale_;
    }

    float Chassis::GetCurrentScale() const {
        if (use_lqr_power_limit_ && lqr_power_limit_) {
            return lqr_power_limit_->GetCurrentScale();
        }
        return 1.0f;
    }
    
    void Chassis::SetBloodPriority(bool blood_priority) {
        if (use_lqr_power_limit_ && lqr_power_limit_) {
            lqr_power_limit_->SetBloodPriority(blood_priority);
        }
    }
    
    void Chassis::SetSeason(SeasonRuleManager::SeasonType season) {
        if (use_lqr_power_limit_ && lqr_power_limit_) {
            lqr_power_limit_->SetSeason(season);
        }
    }

    ChassisCanBridgeSender::ChassisCanBridgeSender(communication::CanBridge* can_bridge,
                                                   uint8_t rx_id)
        : can_bridge_(can_bridge), device_rx_id_(rx_id) {
        rx_id_.data.rx_id = rx_id;
        rx_id_.data.type = communication::CAN_BRIDGE_TYPE_TWO_FLOAT;
    }
    void ChassisCanBridgeSender::SetChassisRegId(uint8_t xy_reg_id, uint8_t turn_on_reg_id,
                                                 uint8_t power_limit_reg_id,
                                                 uint8_t current_power_reg_id) {
        chassis_xy_reg_id_ = xy_reg_id;
        chassis_turn_on_reg_id_ = turn_on_reg_id;
        chassis_power_limit_reg_id_ = power_limit_reg_id;
        chassis_current_power_reg_id_ = current_power_reg_id;
    }
    void ChassisCanBridgeSender::Enable() {
        if (chassis_enable_) {
            return;
        }
        chassis_enable_ = true;

        // Explicit Enable is transmitted as a complete zero-speed transaction so
        // a sequence-aware receiver can enable without consuming stale XY data.
        motion_sequence_ = (motion_sequence_ + 1U) & 0x07U;
        rx_id_.data.res = motion_sequence_;
        data_.data_two_float.data[0] = 0.0f;
        data_.data_two_float.data[1] = 0.0f;
        rx_id_.data.reg = chassis_xy_reg_id_;
        can_bridge_->Send(rx_id_, data_);
        data_.data_two_float.data[0] = 1.0f;
        data_.data_two_float.data[1] = 0.0f;
        rx_id_.data.reg = chassis_turn_on_reg_id_;
        can_bridge_->Send(rx_id_, data_);
    }
    void ChassisCanBridgeSender::Disable() {
        chassis_enable_ = false;
        motion_sequence_ = (motion_sequence_ + 1U) & 0x07U;
        rx_id_.data.res = motion_sequence_;
        data_.data_two_float.data[0] = 0.0f;
        data_.data_two_float.data[1] = 0.0f;
        rx_id_.data.reg = chassis_turn_on_reg_id_;
        can_bridge_->Send(rx_id_, data_);
        chassis_vx = 0.0f;
        chassis_vy = 0.0f;
        chassis_vt = 0.0f;
    }
    void ChassisCanBridgeSender::SetSpeed(const float x_speed, const float y_speed,
                                          const float turn_speed) {
        if (!chassis_enable_) {
            // Reassert the safe state without publishing the requested command.
            data_.data_two_float.data[0] = 0.0f;
            data_.data_two_float.data[1] = 0.0f;
            rx_id_.data.reg = chassis_turn_on_reg_id_;
            can_bridge_->Send(rx_id_, data_);
            chassis_vx = 0.0f;
            chassis_vy = 0.0f;
            chassis_vt = 0.0f;
            return;
        }

        motion_sequence_ = (motion_sequence_ + 1U) & 0x07U;
        rx_id_.data.res = motion_sequence_;

        data_.data_two_float.data[0] = x_speed;
        data_.data_two_float.data[1] = y_speed;
        rx_id_.data.reg = chassis_xy_reg_id_;
        can_bridge_->Send(rx_id_, data_);

        data_.data_two_float.data[0] = 1.0f;
        data_.data_two_float.data[1] = turn_speed;
        rx_id_.data.reg = chassis_turn_on_reg_id_;
        can_bridge_->Send(rx_id_, data_);

        chassis_vx = x_speed;
        chassis_vy = y_speed;
        chassis_vt = turn_speed;
    }
    void ChassisCanBridgeSender::SetPower(bool power_limit_on, float power_limit,
                                          float chassis_power, float chassis_power_buffer,
                                          bool enable_supercap, bool force_update) {
        UNUSED(force_update);
        if (chassis_enable_) {
            {
                chassis_power_limit_on_ = power_limit_on;
                chassis_power_limit_ = power_limit;
                data_.data_eight_uint8.data[0] = power_limit_on;
                data_.data_eight_uint8.data[1] = enable_supercap;
                data_.data_two_float.data[1] = power_limit;
                rx_id_.data.res = 0U;
                rx_id_.data.reg = chassis_power_limit_reg_id_;
                can_bridge_->Send(rx_id_, data_);
            }
            {
                data_.data_two_float.data[0] = chassis_power;
                data_.data_two_float.data[1] = chassis_power_buffer;
                rx_id_.data.res = 0U;
                rx_id_.data.reg = chassis_current_power_reg_id_;
                can_bridge_->Send(rx_id_, data_);
            }
        }
    }
    void ChassisCanBridgeSender::UpdatePowerLimitState(
        bool enabled, uint8_t max_watt, float current_voltage,
        float buffer_joule, bool referee_online) {
        if (!std::isfinite(buffer_joule)) buffer_joule = 0.0f;
        if (buffer_joule < 0.0f) buffer_joule = 0.0f;
        if (buffer_joule > 60.0f) buffer_joule = 60.0f;
        if (!std::isfinite(current_voltage) || current_voltage <= 1.0f) {
            current_voltage = 0.0f;
        }

        data_.data_eight_uint8.data[0] = enabled ? 1U : 0U;
        data_.data_eight_uint8.data[1] = max_watt;
        data_.data_eight_uint8.data[2] =
            static_cast<uint8_t>(buffer_joule + 0.5f);
        data_.data_eight_uint8.data[3] = referee_online ? 1U : 0U;
        data_.data_two_float.data[1] = current_voltage;
        rx_id_.data.res = 0U;
        rx_id_.data.reg = chassis_power_limit_reg_id_;
        can_bridge_->Send(rx_id_, data_);
    }

    void ChassisCanBridgeSender::UpdateCurrentPower(float measured_power,
                                                     float buffer_joule) {
        if (!std::isfinite(buffer_joule)) buffer_joule = 0.0f;
        if (buffer_joule < 0.0f) buffer_joule = 0.0f;
        if (buffer_joule > 60.0f) buffer_joule = 60.0f;
        if (!std::isfinite(measured_power) || measured_power < 0.0f) {
            measured_power = -1.0f;
        }

        data_.data_two_float.data[0] = measured_power;
        data_.data_two_float.data[1] = buffer_joule;
        rx_id_.data.res = 0U;
        rx_id_.data.reg = chassis_current_power_reg_id_;
        can_bridge_->Send(rx_id_, data_);
    }

    void ChassisCanBridgeSender::UpdatePower(
        bool enabled, uint8_t max_watt, float current_voltage,
        float buffer_joule, float measured_power, bool referee_online) {
        // Power transport is independent of motion enable/disable.  The
        // dedicated Atlas PowerTask therefore keeps updating the lower
        // board even while the chassis motion sender is disabled.
        UpdatePowerLimitState(enabled, max_watt, current_voltage, buffer_joule,
                              referee_online);
        UpdateCurrentPower(measured_power, buffer_joule);
    }

    void ChassisCanBridgeSender::UpdatePower(bool enabled, uint8_t max_watt,
                                             float current_voltage,
                                             uint8_t buffer_percent) {
        const float buffer_joule =
            60.0f * static_cast<float>(buffer_percent > 100 ? 100
                                                            : buffer_percent) /
            100.0f;
        UpdatePower(enabled, max_watt, current_voltage, buffer_joule, -1.0f,
                    false);
    }

} /* namespace control */
