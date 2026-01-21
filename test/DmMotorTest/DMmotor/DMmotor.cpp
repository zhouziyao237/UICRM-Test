// Copyright (c) 2025. BNU-HKBU UIC RoboMaster
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

#include "DMmotor.h"

namespace driver {

    // CAN回调函数，用于更新电机反馈数据
    static void can_motor_callback(const uint8_t data[], void* args) {
        DMMotor* motor = reinterpret_cast<DMMotor*>(args);
        motor->UpdateData(data);
    }

    DMMotor::DMMotor(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id, dm_motor_mode_t mode)
        : ConnectionDriver(50),
        can_(can),
        rx_id_(rx_id),
        tx_id_base_(tx_id),
        mode_(mode) {

        can->RegisterRxCallback(rx_id, can_motor_callback, this);

        // 根据模式计算实际发送ID（参考说明书第6-7页）
        switch (mode) {
            case MIT:
                tx_id_actual_ = tx_id_base_;
                break;
            case POS_VEL:
                tx_id_actual_ = tx_id_base_ + 0x100;
                break;
            case VEL:
                tx_id_actual_ = tx_id_base_ + 0x200;
                break;
            default:
                // 无效模式，触发断言
                RM_EXPECT_TRUE(false, "Invalid motor mode!");
                break;
        }
    }

    void DMMotor::MotorEnable() {
        uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
        can_->Transmit(tx_id_actual_, data, 8);
    }

    void DMMotor::MotorDisable() {
        uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
        can_->Transmit(tx_id_actual_, data, 8);
    }

    void DMMotor::SetZeroPosition() {
        uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};
        can_->Transmit(tx_id_actual_, data, 8);
    }

    void DMMotor::SetMITParams(float position, float velocity, float kp, float kd, float torque) {
        set_position_ = position;
        set_velocity_ = velocity;
        set_kp_ = kp;
        set_kd_ = kd;
        set_torque_ = torque;
    }

    void DMMotor::SetPositionVelocityParams(float position, float velocity) {
        set_position_ = position;
        set_velocity_ = velocity;
    }

    void DMMotor::SetVelocityParams(float velocity) {
        set_velocity_ = velocity;
    }

    void DMMotor::TransmitOutput() {
        uint8_t data[8] = {0};

        if (mode_ == MIT) {
            // MIT模式控制帧（参考说明书第6页）
            uint16_t pos_int = FloatToUint(set_position_, POS_MIN, POS_MAX, 16);
            uint16_t vel_int = FloatToUint(set_velocity_, VEL_MIN, VEL_MAX, 12);
            uint16_t kp_int = FloatToUint(set_kp_, KP_MIN, KP_MAX, 12);
            uint16_t kd_int = FloatToUint(set_kd_, KD_MIN, KD_MAX, 12);
            uint16_t torque_int = FloatToUint(set_torque_, TORQUE_MIN, TORQUE_MAX, 12);

            data[0] = pos_int >> 8;
            data[1] = pos_int & 0xFF;
            data[2] = (vel_int >> 4) & 0xFF;
            data[3] = ((vel_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F);
            data[4] = kp_int & 0xFF;
            data[5] = (kd_int >> 4) & 0xFF;
            data[6] = ((kd_int & 0x0F) << 4) | ((torque_int >> 8) & 0x0F);
            data[7] = torque_int & 0xFF;

        } else if (mode_ == POS_VEL) {
            // 位置速度模式控制帧（浮点数，低位在前）
            // 修复：使用临时变量去除 volatile 限定符
            float temp_pos = set_position_;
            float temp_vel = set_velocity_;
            uint8_t *pos_ptr = reinterpret_cast<uint8_t*>(&temp_pos);
            uint8_t *vel_ptr = reinterpret_cast<uint8_t*>(&temp_vel);

            for (int i = 0; i < 4; i++) {
                data[i] = pos_ptr[i];
                data[i + 4] = vel_ptr[i];
            }

        } else if (mode_ == VEL) {
            // 速度模式控制帧（浮点数，低位在前）
            // 修复：使用临时变量去除 volatile 限定符
            float temp_vel = set_velocity_;
            uint8_t *vel_ptr = reinterpret_cast<uint8_t*>(&temp_vel);

            for (int i = 0; i < 4; i++) {
                data[i] = vel_ptr[i];
            }
        } else {
            RM_EXPECT_TRUE(false, "Invalid motor mode in SendControlCommand");
        }

        can_->Transmit(tx_id_actual_, data, 8);
    }

    void DMMotor::UpdateData(const uint8_t data[]) {
        // 反馈帧格式解析（参考说明书第6页）
        motor_id_ = data[0] & 0x0F;
        error_state_ = data[0] >> 4;
        raw_position_ = (data[1] << 8) | data[2];
        raw_velocity_ = (data[3] << 4) | (data[4] >> 4);
        raw_torque_ = ((data[4] & 0x0F) << 8) | data[5];
        raw_mos_temp_ = data[6];
        raw_motor_temp_ = data[7];

        // 转换为物理量
        current_position_ = UintToFloat(raw_position_, POS_MIN, POS_MAX, 16);
        current_velocity_ = UintToFloat(raw_velocity_, VEL_MIN, VEL_MAX, 12);
        current_torque_ = UintToFloat(raw_torque_, TORQUE_MIN, TORQUE_MAX, 12);

        Heartbeat();  // 更新在线状态
    }

    void DMMotor::PrintData() {
        print("MotorID: %d ", motor_id_);
        print("ErrorState: 0x%X ", error_state_);
        print("Online: %s ", (IsOnline() ? "Yes" : "No"));
        print("Position: %.4f rad ", current_position_);
        print("Velocity: %.4f rad/s ", current_velocity_);
        print("Torque: %.4f Nm ", current_torque_);
        print("MOS Temp: %d C ", raw_mos_temp_);
        print("Rotor Temp: %d C \r\n", raw_motor_temp_);
    }

    uint16_t DMMotor::FloatToUint(float x, float x_min, float x_max, int bits) {
        float span = x_max - x_min;
        float offset = x_min;
        float normalized = (x - offset) / span;
        return static_cast<uint16_t>(normalized * ((1 << bits) - 1));
    }

    float DMMotor::UintToFloat(int x_int, float x_min, float x_max, int bits) {
        float span = x_max - x_min;
        float offset = x_min;
        float normalized = static_cast<float>(x_int) / ((1 << bits) - 1);
        return normalized * span + offset;
    }

    // Getter 实现
    float DMMotor::GetPosition() const { return current_position_; }
    float DMMotor::GetVelocity() const { return current_velocity_; }
    float DMMotor::GetTorque() const { return current_torque_; }
    int DMMotor::GetErrorState() const { return error_state_; }
    int DMMotor::GetMotorID() const { return motor_id_; }

}  // namespace driver