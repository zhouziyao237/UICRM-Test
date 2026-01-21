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

#include "arm_math.h"
#include "bsp_os.h"
#include "utils.h"
#include "bsp_thread.h"
#include "bsp_error_handler.h"

namespace driver {

    // ==================== 静态成员初始化 ====================
    bool DMMotor::is_init_ = false;
    bsp::Thread* DMMotor::control_thread_ = nullptr;
    uint32_t DMMotor::control_freq_ = 500;  // 默认500Hz
    uint32_t DMMotor::control_interval_ = 1000 / 500;  // 2ms
    DMMotor::MotorGroup DMMotor::motor_groups_[DMMotor::MAX_GROUPS];
    uint8_t DMMotor::group_count_ = 0;

    // ==================== CAN回调函数 ====================
    static void can_motor_callback(const uint8_t data[], void* args) {
        DMMotor* motor = reinterpret_cast<DMMotor*>(args);
        motor->UpdateData(data);
    }

    // ==================== 构造函数 ====================
    DMMotor::DMMotor(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id, dm_motor_mode_t mode,
                     DMMotorProtectConfig config)
        : ConnectionDriver(50),
          can_(can),
          rx_id_(rx_id),
          tx_id_base_(tx_id),
          mode_(mode),
          ctrl_state_(DISABLED),
          protect_config_(config){

        // 计算实际发送ID
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
                RM_EXPECT_TRUE(false, "Invalid motor mode!");
                break;
        }

        // 注册CAN回调
        can->RegisterRxCallback(rx_id, can_motor_callback, this);

        // 初始化控制线程（第一次创建电机时）
        if (!is_init_) {
            control_interval_ = 1000 / control_freq_;  // 计算间隔时间
            bsp::thread_init_t thread_init = {
                .func = DMMotor::ControlThread,
                .args = nullptr,
                .attr = DMMotor::thread_attr_
            };
            control_thread_ = new bsp::Thread(thread_init);
            control_thread_->Start();
            is_init_ = true;
        }

        // 注册到控制线程
        RegisterToControlThread();
    }

    // ==================== 线程注册 ====================
    void DMMotor::RegisterToControlThread() {
        // 检查是否已存在相同ID的电机
        for (uint8_t i = 0; i < group_count_; i++) {
            if (motor_groups_[i].tx_id == tx_id_actual_ &&
                motor_groups_[i].can == can_) {
                // 不允许重复ID（达妙电机每个ID独立）
                RM_ASSERT_TRUE(false, "Duplicate motor ID detected!");
                return;
            }
        }

        // 添加到控制组
        if (group_count_ < MAX_GROUPS) {
            motor_groups_[group_count_].can = can_;
            motor_groups_[group_count_].tx_id = tx_id_actual_;
            motor_groups_[group_count_].motor = this;
            motor_groups_[group_count_].active = true;
            group_count_++;
        } else {
            RM_ASSERT_TRUE(false, "Exceeded maximum number of motor groups!");
        }
    }

    // ==================== 控制线程 ====================
    void DMMotor::ControlThread(void* args) {
        (void)args;  // 未使用

        while (true) {
            // 遍历所有电机组并发送控制指令
            for (uint8_t i = 0; i < group_count_; i++) {
                if (motor_groups_[i].active && motor_groups_[i].motor) {
                    DMMotor* motor = motor_groups_[i].motor;

                    // 检查防护条件
                    if (motor->CheckProtection()) {
                        // 如果存在故障，跳过发送控制指令
                        continue;
                    }

                    // 打包控制数据
                    uint8_t data[8];
                    motor->PackControlData(data);

                    // 发送CAN帧
                    motor_groups_[i].can->Transmit(motor_groups_[i].tx_id, data, 8);
                }
            }

            // 固定频率延时
            osDelay(control_interval_);
        }
    }

    // ==================== 防护检测 ====================
    bool DMMotor::CheckProtection() {
        // 如果控制侧已锁定故障，直接返回true
        if (ctrl_state_ == FAULT && fault_latched_) {
            return true;
        }

        // 1. 检查硬件反馈的故障状态
        if (hw_state_ >= DM_FAULT_OVERVOLT) {  // 0x8及以上是故障状态
            // 硬件报告故障，需要处理
            HandleFault(hw_state_);
            return true;
        }

        // 2. 检查通讯丢失
        if (!IsOnline()) {
            uint32_t current_time = bsp::GetHighresTickMicroSec();

            if (comm_loss_time == 0) {
                comm_loss_time = current_time;
            } else if (current_time - comm_loss_time > 200) {  // 200ms通讯丢失
                HandleFault(DM_FAULT_COMM_LOSS);
                return true;
            }
        } else {
            // 通讯正常，重置计时
            comm_loss_time = 0;
        }

        // 3. 检查软件侧的过载保护（只在电机使能状态下检查）
        if (hw_state_ == DM_STATE_ENABLED && ctrl_state_ >= WAITING) {
            float abs_torque = fabsf(current_torque_);
            if (abs_torque > protect_config_.overcurrent_threshold) {
                uint32_t current_time = bsp::GetHighresTickMicroSec();
                if (overload_start_time_ == 0) {
                    overload_start_time_ = current_time;
                } else if (current_time - overload_start_time_ > protect_config_.overload_duration) {
                    HandleFault(DM_FAULT_OVERLOAD);
                    return true;
                }
            } else {
                overload_start_time_ = 0;
            }
        }

        return false;
    }

    void DMMotor::HandleFault(dm_motor_state_t fault) {
        // 锁定故障状态
        ctrl_state_ = FAULT;
        fault_latched_ = true;
        latched_fault_ = fault;

        // 调用故障回调函数
        if (protect_config_.fault_callback != nullptr) {
            protect_config_.fault_callback(protect_config_.fault_callback_instance, fault);
        }

        // 自动失能电机（安全措施）
        MotorDisable();
    }

    bool DMMotor::ClearFault() {
        if (ctrl_state_ == FAULT && fault_latched_) {
            // 清除故障锁定
            fault_latched_ = false;
            latched_fault_ = DM_STATE_DISABLED;

            // 重置到失能状态
            ctrl_state_ = DISABLED;
            hw_state_ = DM_STATE_DISABLED;

            // 重置防护计时器
            overload_start_time_ = 0;

            return true;
        }
        return false;
    }

    bool DMMotor::IsInFault() const {
        return (ctrl_state_ == FAULT && fault_latched_) ||
               (hw_state_ >= DM_FAULT_OVERVOLT);
    }

    // ==================== 数据打包 ====================
    void DMMotor::PackControlData(uint8_t data[8]) {
        // 如果电机处于故障状态，发送停止指令
        if (ctrl_state_ == FAULT || hw_state_ >= DM_FAULT_OVERVOLT) {
            for (int i = 0; i < 8; i++) {
                data[i] = 0;
            }
            return;
        }

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
            float temp_pos = set_position_;
            float temp_vel = set_velocity_;
            uint8_t* pos_ptr = reinterpret_cast<uint8_t*>(&temp_pos);
            uint8_t* vel_ptr = reinterpret_cast<uint8_t*>(&temp_vel);

            for (int i = 0; i < 4; i++) {
                data[i] = pos_ptr[i];
                data[i + 4] = vel_ptr[i];
            }

        } else if (mode_ == VEL) {
            // 速度模式控制帧（浮点数，低位在前）
            float temp_vel = set_velocity_;
            uint8_t* vel_ptr = reinterpret_cast<uint8_t*>(&temp_vel);

            for (int i = 0; i < 4; i++) {
                data[i] = vel_ptr[i];
            }
        } else {
            // 无效模式，发送停止指令
            for (int i = 0; i < 8; i++) {
                data[i] = 0;
            }
        }
    }

    // ==================== 控制频率设置 ====================
    void DMMotor::SetControlFrequency(uint32_t freq) {
        RM_ASSERT_FALSE(is_init_, "Frequency must be set before creating any motor!");
        RM_ASSERT_GE(freq, 100, "Control频率太低 (最低100Hz)");
        RM_ASSERT_LE(freq, 1000, "控制频率太高 (最高1000Hz)");

        control_freq_ = freq;
        control_interval_ = 1000 / freq;
    }

    // ==================== 基本控制函数 ====================
    void DMMotor::MotorEnable() {
        if (ctrl_state_ == FAULT && fault_latched_) {
            // 故障状态下不能使能
            return;
        }

        uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
        can_->Transmit(tx_id_actual_, data, 8);

        // 更新控制侧状态
        ctrl_state_ = ENABLED;
    }

    void DMMotor::MotorDisable() {
        uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
        can_->Transmit(tx_id_actual_, data, 8);

        // 更新控制侧状态
        ctrl_state_ = DISABLED;
    }

    void DMMotor::SetZeroPosition() {
        uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};
        can_->Transmit(tx_id_actual_, data, 8);
    }

    // ==================== 参数设置函数 ====================
    void DMMotor::SetMITParams(float position, float velocity, float kp, float kd, float torque) {
        if (ctrl_state_ == FAULT || hw_state_ >= DM_FAULT_OVERVOLT) {
            return;  // 故障状态下不接受参数设置
        }

        set_position_ = position;
        set_velocity_ = velocity;
        set_kp_ = kp;
        set_kd_ = kd;
        set_torque_ = torque;

        // 只有在电机使能状态下才更新为等待状态
        if (ctrl_state_ == ENABLED) {
            ctrl_state_ = WAITING;
        }
    }

    void DMMotor::SetPositionVelocityParams(float position, float velocity) {
        if (ctrl_state_ == FAULT || hw_state_ >= DM_FAULT_OVERVOLT) {
            return;
        }

        set_position_ = position;
        set_velocity_ = velocity;

        if (ctrl_state_ == ENABLED) {
            ctrl_state_ = WAITING;
        }
    }

    void DMMotor::SetVelocityParams(float velocity) {
        if (ctrl_state_ == FAULT || hw_state_ >= DM_FAULT_OVERVOLT) {
            return;
        }

        set_velocity_ = velocity;

        if (ctrl_state_ == ENABLED) {
            ctrl_state_ = WAITING;
        }
    }

    // ==================== 数据更新 ====================
    void DMMotor::UpdateData(const uint8_t data[]) {
        // 反馈帧格式解析（参考说明书第6页）
        motor_id_ = data[0] & 0x0F;

        // 解析硬件状态
        uint8_t err_byte = data[0] >> 4;
        hw_state_ = static_cast<dm_motor_state_t>(err_byte);

        raw_position_ = (data[1] << 8) | data[2];
        raw_velocity_ = (data[3] << 4) | (data[4] >> 4);
        raw_torque_ = ((data[4] & 0x0F) << 8) | data[5];
        raw_mos_temp_ = data[6];
        raw_motor_temp_ = data[7];

        // 转换为物理量
        current_position_ = UintToFloat(raw_position_, POS_MIN, POS_MAX, 16);
        current_velocity_ = UintToFloat(raw_velocity_, VEL_MIN, VEL_MAX, 12);
        current_torque_ = UintToFloat(raw_torque_, TORQUE_MIN, TORQUE_MAX, 12);

        // 更新控制侧状态（基于运动状态）
        if (ctrl_state_ >= WAITING && ctrl_state_ != FAULT) {
            if (fabsf(current_velocity_) < 0.01f) {  // 速度接近0
                if (ctrl_state_ == MOVING || ctrl_state_ == WAITING) {
                    ctrl_state_ = STABLE;
                }
            } else {
                if (ctrl_state_ == WAITING || ctrl_state_ == STABLE) {
                    ctrl_state_ = MOVING;
                }
            }
        }

        Heartbeat();  // 更新在线状态
    }

    // ==================== 调试输出 ====================
    void DMMotor::PrintData() {
        const char* ctrl_state_str[] = {
            "DISABLED", "ENABLED", "WAITING",
            "MOVING", "STABLE", "FAULT"
        };

        const char* hw_state_str[] = {
            "DISABLED", "ENABLED", "", "", "", "", "", "",
            "OVERVOLT", "UNDERVOLT", "OVERCURRENT", "MOS_OVERTEMP",
            "MOTOR_OVERTEMP", "COMM_LOSS", "OVERLOAD"
        };

        print("DM Motor ID:%d ", motor_id_);
        print("HW:%s SW:%s ",
              hw_state_str[hw_state_],
              ctrl_state_str[ctrl_state_]);
        print("Online:%s ", (IsOnline() ? "Yes" : "No"));
        print("Pos:%.3frad Vel:%.3frad/s Torq:%.3fNm ",
              current_position_, current_velocity_, current_torque_);
        print("MOS:%dC Motor:%dC\r\n",
              raw_mos_temp_, raw_motor_temp_);
    }

    // ==================== 数据转换函数 ====================
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

    // ==================== Getter函数 ====================
    float DMMotor::GetPosition() const { return current_position_; }
    float DMMotor::GetVelocity() const { return current_velocity_; }
    float DMMotor::GetTorque() const { return current_torque_; }
    dm_motor_state_t DMMotor::GetHardwareState() const { return hw_state_; }
    DMMotor::ControlState DMMotor::GetControlState() const { return ctrl_state_; }
    int DMMotor::GetMotorID() const { return motor_id_; }

    DMMotorProtectConfig DMMotor::GetProtectConfig() const { return protect_config_; }

    void DMMotor::SetProtectConfig(const DMMotorProtectConfig& config) {
        protect_config_ = config;
    }

    void DMMotor::RegisterFaultCallback(void (*callback)(void* instance, dm_motor_state_t fault), void* instance) {
        protect_config_.fault_callback = callback;
        protect_config_.fault_callback_instance = instance;
    }

}  // namespace driver