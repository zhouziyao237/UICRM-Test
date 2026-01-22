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
#include "bsp_error_handler.h"
#include "bsp_os.h"
#include "bsp_thread.h"
#include "utils.h"

namespace driver {

    // ==================== 静态成员初始化 ====================
    bool DMMotor::is_init_ = false;
    bsp::Thread* DMMotor::control_thread_ = nullptr;
    uint32_t DMMotor::control_freq_ = 500;             // 默认500Hz
    uint32_t DMMotor::control_interval_ = 1000 / 500;  // 2ms
    DMMotor::MotorGroup DMMotor::motor_groups_[DMMotor::MAX_GROUPS];
    uint8_t DMMotor::group_count_ = 0;

    // ==================== CAN回调函数 ====================
    static void can_motor_callback(const uint8_t data[], void* args) {
        DMMotor* motor = reinterpret_cast<DMMotor*>(args);
        motor->UpdateData(data);
    }

    static void CanMITCtrlcallback(const uint8_t data[], void* args) {
        DMMotorMITController* motor = reinterpret_cast<DMMotorMITController*>(args);
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
          protect_config_(config) {
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
                .func = DMMotor::ControlThread, .args = nullptr, .attr = DMMotor::thread_attr_};
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
            if (motor_groups_[i].tx_id == tx_id_actual_ && motor_groups_[i].can == can_) {
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
        UNUSED(args);  // 未使用

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

                    if (motor->MITController) {
                        motor->TorqueOutput();
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
                } else if (current_time - overload_start_time_ >
                           protect_config_.overload_duration) {
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
        return (ctrl_state_ == FAULT && fault_latched_) || (hw_state_ >= DM_FAULT_OVERVOLT);
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
        motor_id_ = data[0] & 0x0F;  // 电机ID (0-15)

        // 解析硬件状态
        uint8_t err_byte = data[0] >> 4;                      // 错误状态字节 (高4位)
        hw_state_ = static_cast<dm_motor_state_t>(err_byte);  // 硬件状态 (使能/故障状态)

        raw_position_ = (data[1] << 8) | data[2];         // 原始位置数据 (16位整数)
        raw_velocity_ = (data[3] << 4) | (data[4] >> 4);  // 原始速度数据 (12位整数)
        raw_torque_ = ((data[4] & 0x0F) << 8) | data[5];  // 原始扭矩数据 (12位整数)
        raw_mos_temp_ = data[6];                          // MOS管温度 (摄氏度)
        raw_motor_temp_ = data[7];                        // 电机温度 (摄氏度)

        // 转换为物理量
        current_position_ =
            UintToFloat(raw_position_, POS_MIN, POS_MAX, 16);  // 当前位置 (弧度, rad)
        current_velocity_ =
            UintToFloat(raw_velocity_, VEL_MIN, VEL_MAX, 12);  // 当前速度 (弧度/秒, rad/s)
        current_torque_ =
            UintToFloat(raw_torque_, TORQUE_MIN, TORQUE_MAX, 12);  // 当前扭矩 (牛米, Nm)

        // 更新控制侧状态（基于运动状态）
        if (ctrl_state_ >= WAITING && ctrl_state_ != FAULT) {
            if (fabsf(current_velocity_) < 0.01f) {  // 速度接近0 (小于0.01 rad/s)
                if (ctrl_state_ == MOVING || ctrl_state_ == WAITING) {
                    ctrl_state_ = STABLE;  // 切换到稳定状态
                }
            } else {
                if (ctrl_state_ == WAITING || ctrl_state_ == STABLE) {
                    ctrl_state_ = MOVING;  // 切换到运动状态
                }
            }
        }

        Heartbeat();  // 更新在线状态
    }

    // ==================== 调试输出 ====================
    void DMMotor::PrintData() {
        const char* ctrl_state_str[] = {"DISABLED", "ENABLED", "WAITING",
                                        "MOVING",   "STABLE",  "FAULT"};

        const char* hw_state_str[] = {"DISABLED",
                                      "ENABLED",
                                      "",
                                      "",
                                      "",
                                      "",
                                      "",
                                      "",
                                      "OVERVOLT",
                                      "UNDERVOLT",
                                      "OVERCURRENT",
                                      "MOS_OVERTEMP",
                                      "MOTOR_OVERTEMP",
                                      "COMM_LOSS",
                                      "OVERLOAD"};

        print("DM Motor ID:%d ", motor_id_);
        print("HW:%s SW:%s ", hw_state_str[hw_state_], ctrl_state_str[ctrl_state_]);
        print("Online:%s ", (IsOnline() ? "Yes" : "No"));
        print("Pos:%.3frad Vel:%.3frad/s Torq:%.3fNm ", current_position_, current_velocity_,
              current_torque_);
        print("MOS:%dC Motor:%dC\r\n", raw_mos_temp_, raw_motor_temp_);
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

    void DMMotor::TorqueOutput() {
    }

    // ==================== Getter函数 ====================
    float DMMotor::GetPosition() const {
        return current_position_;
    }
    float DMMotor::GetVelocity() const {
        return current_velocity_;
    }
    float DMMotor::GetTorque() const {
        return current_torque_;
    }
    dm_motor_state_t DMMotor::GetHardwareState() const {
        return hw_state_;
    }
    DMMotor::ControlState DMMotor::GetControlState() const {
        return ctrl_state_;
    }
    int DMMotor::GetMotorID() const {
        return motor_id_;
    }

    DMMotorProtectConfig DMMotor::GetProtectConfig() const {
        return protect_config_;
    }

    void DMMotor::SetProtectConfig(const DMMotorProtectConfig& config) {
        protect_config_ = config;
    }

    void DMMotor::RegisterFaultCallback(void (*callback)(void* instance, dm_motor_state_t fault),
                                        void* instance) {
        protect_config_.fault_callback = callback;
        protect_config_.fault_callback_instance = instance;
    }

    // --------------------------------- DMMotorMITController -------------------------------------

    DMMotorMITController::DMMotorMITController(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id,
                                               DMMotorProtectConfig config)
        : DMMotor(can, rx_id, tx_id, MIT, config) {
        // 初始化变量 - 与DJI电机保持一致
        output_shaft_theta_ = 0;
        output_shaft_omega_ = 0;
        target_ = 0;
        output_ = 0;
        holding_ = false;
        mode_ = NONE;
        speed_offset_ = 0;
        transmission_ratio_ = 1.0f;
        proximity_in_ = 0.05f;  // 5度
        proximity_out_ = 0.1f;  // 10度

        // 默认PID参数
        omega_pid_ = control::ConstrainedPID();
        theta_pid_ = control::ConstrainedPID();

        // 参考DJI做法：使用-1作为未初始化标记
        align_angle_ = -1;  // 等待UpdateData初始化
        motor_angle_ = 0;
        offset_angle_ = 0;
        servo_angle_ = 0;
        cumulated_angle_ = 0;

        // 注意：达妙电机范围是[-12.5, 12.5]，但实际编码器输出经过包装后是[-π, π]
        // FloatEdgeDetector使用[0, π]范围进行边界检测（与DJI保持一致）
        inner_wrap_detector_ = new FloatEdgeDetector(0, PI);
        outer_wrap_detector_ = new FloatEdgeDetector(0, PI);

        can->RegisterRxCallback(rx_id, CanMITCtrlcallback, this);

        MITController = true;
    }

    void DMMotorMITController::SetTarget(float target, bool override) {
        // 设置目标值
        if (mode_ & REVERSE_MOTOR_OPERATE) {
            target = -target;
        }
        if (override == false && !holding_) {
            // 如果电机没有在hold状态，则不修改目标值
            return;
        }
        target_ = target;

        if ((mode_ & ANGLE_LOOP_CONTROL) && (mode_ & ABSOLUTE)) {
            target_ = wrap<float>(target_, -PI, PI);
        }

        // 保持状态检测（仅在角度环模式下）
        if (mode_ & ANGLE_LOOP_CONTROL) {
            float current_theta = GetOutputShaftTheta();
            float angle_diff = 0;

            if (mode_ & ABSOLUTE) {
                // 绝对模式：计算包装后的角度差
                angle_diff = wrap<float>(target_ - current_theta, -PI, PI);
            } else {
                // 相对模式：直接计算差值
                angle_diff = target_ - current_theta;
            }

            if (!holding_ && fabsf(angle_diff) <= proximity_in_) {
                holding_ = true;
            } else if (holding_ && fabsf(angle_diff) > proximity_out_) {
                holding_ = false;
            }
        }
    }

    void DMMotorMITController::SetOutput(float torque) {
        output_ = torque;
        SetMITParams(0, 0, 0, 0, output_);
    }

    // ==================== TorqueOutput函数修正（达妙专用） ====================
    void DMMotorMITController::TorqueOutput() {
        // 检查电机状态
        if (DMMotor::GetControlState() == DISABLED) {
            SetOutput(0);
            theta_pid_.ResetIntegral();
            omega_pid_.ResetIntegral();
            return;
        }

        float torque_output = 0.0f;  // 最终输出的扭矩

        // 1. 速度环控制（直接输出扭矩）
        if (mode_ & SPEED_LOOP_CONTROL) {
            // 获取当前输出轴速度
            float current_omega = GetOutputShaftOmega();

            // 速度目标值
            float speed_target = target_;

            // 加上速度前馈偏移
            speed_target += speed_offset_;

            // 计算速度环PID输出（直接输出扭矩，单位：Nm）
            torque_output = omega_pid_.ComputeOutput(speed_target, current_omega);
        }

        // 2. 角度环控制（直接输出扭矩）
        if (mode_ & ANGLE_LOOP_CONTROL) {
            // 获取当前输出轴角度
            float current_theta = GetOutputShaftTheta();

            // 角度目标值
            float angle_target = target_;

            if (mode_ & ABSOLUTE) {
                // 绝对模式：计算最短路径
                float angle_diff = angle_target - current_theta;

                // 包装到 [-π, π] 范围，确保走最短路径
                float wrapped_diff = wrap<float>(angle_diff, -PI, PI);

                // 使用包装后的角度作为目标
                angle_target = current_theta + wrapped_diff;
            }

            // 计算角度环PID输出（直接输出扭矩，单位：Nm）
            // 注意：这里和MotorCanBase一样，角度环PID输出直接就是扭矩
            float angle_torque = theta_pid_.ComputeOutput(angle_target, current_theta);

            // 如果同时开启了速度环，则角度环输出会叠加到速度环输出上
            if (mode_ & SPEED_LOOP_CONTROL) {
                torque_output += angle_torque;
            } else {
                // 如果只有角度环，直接使用角度环输出
                torque_output = angle_torque;
            }
        }

        // 3. 如果没有启用任何控制环，输出0
        if (mode_ == NONE) {
            torque_output = 0;
        }

        // 4. 设置扭矩输出
        SetOutput(torque_output);
    }

    void DMMotorMITController::ReInitPID(control::ConstrainedPID::PID_Init_t pid_init,
                                         uint8_t mode) {
        if (mode & SPEED_LOOP_CONTROL) {
            omega_pid_.Reinit(pid_init);
        } else if (mode & ANGLE_LOOP_CONTROL) {
            theta_pid_.Reinit(pid_init);
        }
    }

    control::ConstrainedPID::PID_State_t DMMotorMITController::GetPIDState(uint8_t mode) const {
        if (mode & SPEED_LOOP_CONTROL && mode_ & SPEED_LOOP_CONTROL) {
            return omega_pid_.State();
        } else if (mode & ANGLE_LOOP_CONTROL && mode_ & ANGLE_LOOP_CONTROL) {
            return theta_pid_.State();
        }
        return control::ConstrainedPID::PID_State_t();
    }

    void DMMotorMITController::SetMode(uint8_t mode) {
        mode_ = mode;
    }

    void DMMotorMITController::UpdateData(const uint8_t* data) {
        UNUSED(data);
        DMMotor::UpdateData(data);

        // 将达妙电机的角度范围[-12.5, 12.5]转换为[0, 2π]范围用于记圈
        float damiao_angle = DMMotor::current_position_;
        float mapped_angle = (damiao_angle + DAMIAO_HALF_RANGE) * (2 * PI) / DAMIAO_RANGE;

        if (align_angle_ < 0) {
            align_angle_ = mapped_angle;
        }

        // 计算相对于对齐角度的差值，映射到[0, 2π]范围
        float relative_angle = wrap<float>(mapped_angle - align_angle_, 0, 2 * PI);

        if (transmission_ratio_ != 1) {
            // 使用[0, 2π]范围的角度进行边界检测
            inner_wrap_detector_->input(relative_angle);

            if (inner_wrap_detector_->negEdge()) {
                // 从接近2π跳到接近0，电机正向转过一圈
                offset_angle_ =
                    wrap<float>(offset_angle_ + 2 * PI / transmission_ratio_, 0, 2 * PI);
            } else if (inner_wrap_detector_->posEdge()) {
                // 从接近0跳到接近2π，电机反向转过一圈
                offset_angle_ =
                    wrap<float>(offset_angle_ - 2 * PI / transmission_ratio_, 0, 2 * PI);
            }

            servo_angle_ =
                wrap<float>(offset_angle_ + relative_angle / transmission_ratio_, 0, 2 * PI);
        } else {
            servo_angle_ = wrap<float>(relative_angle, 0, 2 * PI);
        }

        // 外部边界检测用于累计多圈角度
        outer_wrap_detector_->input(servo_angle_);

        // 累计角度处理（仅在非绝对模式下）
        if (!(mode_ & ABSOLUTE)) {
            if (outer_wrap_detector_->negEdge()) {
                // 从接近2π跳到接近0，输出轴正向转过一圈
                cumulated_angle_ += 2 * PI;
            } else if (outer_wrap_detector_->posEdge()) {
                // 从接近0跳到接近2π，输出轴反向转过一圈
                cumulated_angle_ -= 2 * PI;
            }
        }

        // 速度转换：同样使用映射方法
        float damiao_velocity = DMMotor::current_velocity_;
        float mapped_velocity = damiao_velocity * (2 * PI) / DAMIAO_RANGE;

        // 计算最终输出轴角度（包含累计角度）
        output_shaft_theta_ = servo_angle_ + cumulated_angle_;
        output_shaft_omega_ = mapped_velocity / transmission_ratio_;

        if (mode_ & ANGLE_LOOP_CONTROL) {
            // 控制逻辑中仍然使用用户友好的[-π, π]范围
            float user_friendly_angle = wrap<float>(output_shaft_theta_, -PI, PI);

            if (!(mode_ & ABSOLUTE)) {
                if (!holding_ && abs(target_ - user_friendly_angle) <= proximity_in_) {
                    holding_ = true;
                } else if (holding_ && abs(target_ - user_friendly_angle) > proximity_out_) {
                    holding_ = false;
                }
            } else {
                if (!holding_ &&
                    abs(wrap<float>(target_ - user_friendly_angle, -PI, PI)) <= proximity_in_) {
                    holding_ = true;
                } else if (holding_ && abs(wrap<float>(target_ - user_friendly_angle, -PI, PI)) >
                                           proximity_out_) {
                    holding_ = false;
                }
            }
        }
    }

    void DMMotorMITController::SetTransmissionRatio(float ratio) {
        // 设置电机的传动比
        // 这里的传动比不是电机的实际传动比，而是电机与编码器的传动比
        transmission_ratio_ = ratio;
    }

    float DMMotorMITController::GetOutputShaftTheta() const {
        return output_shaft_theta_;
    }

    float DMMotorMITController::GetOutputShaftOmega() const {
        return output_shaft_omega_;
    }

    float DMMotorMITController::GetTarget() const {
        return target_;
    }

    void DMMotorMITController::Enable() {
        DMMotor::MotorEnable();
    }
    void DMMotorMITController::Disable() {
        DMMotor::MotorDisable();
    }

    bool DMMotorMITController::IsEnable() const {
        return DMMotor::GetControlState() == ENABLED ? true : false;
    }

    bool DMMotorMITController::IsHolding() const {
        return holding_;
    }

    void DMMotorMITController::Hold(bool override) {
        if (!IsHolding() && mode_ & ANGLE_LOOP_CONTROL) {
            SetTarget(GetOutputShaftTheta(), override);
        }
    }

    void DMMotorMITController::SetSpeedOffset(float offset) {
        speed_offset_ = offset;
    }

    // ==================== 角度转换方法 ====================
    float DMMotorMITController::StandardToDamiao(float standard_angle) const {
        // 标准角度 [0, 2π] -> 达妙角度 [-12.5, 12.5]
        // 先包装到 [0, 2π]
        float wrapped_angle = wrap<float>(standard_angle, 0, 2 * PI);

        // 将 [0, 2π] 映射到 [-12.5, 12.5]
        // 映射公式：达妙角度 = (标准角度 / (2π)) * 25.0 - 12.5
        constexpr float SCALE_FACTOR = DAMIAO_RANGE / (2 * PI);
        return wrapped_angle * SCALE_FACTOR - DAMIAO_HALF_RANGE;
    }

    float DMMotorMITController::DamiaoToStandard(float damiao_angle) const {
        // 达妙角度 [-12.5, 12.5] -> 标准角度 [0, 2π]
        // 先包装到达妙角度范围
        float wrapped_damiao = WrapDamiaoAngle(damiao_angle);

        // 将 [-12.5, 12.5] 映射到 [0, 2π]
        // 映射公式：标准角度 = (达妙角度 + 12.5) / 25.0 * (2π)
        constexpr float SCALE_FACTOR = (2 * PI) / DAMIAO_RANGE;
        return (wrapped_damiao + DAMIAO_HALF_RANGE) * SCALE_FACTOR;
    }

    float DMMotorMITController::WrapDamiaoAngle(float angle) const {
        // 包装到达妙角度范围 [-12.5, 12.5]
        constexpr float MIN_ANGLE = -DAMIAO_HALF_RANGE;
        constexpr float MAX_ANGLE = DAMIAO_HALF_RANGE;
        constexpr float RANGE = DAMIAO_RANGE;

        // 确保角度在范围内
        while (angle > MAX_ANGLE) {
            angle -= RANGE;
        }
        while (angle < MIN_ANGLE) {
            angle += RANGE;
        }

        return angle;
    }

    // ==================================== 暂存未使用函数 =====================================
    void DMMotorMITController::SetProximityThreshold(float proximity_in, float proximity_out) {
        proximity_in_ = proximity_in;
        proximity_out_ = proximity_out;
    }

    float DMMotorMITController::GetOutput() const {
        return output_;
    }

    float DMMotorMITController::GetThetaDelta(float target) const {
        float diff = target - output_shaft_theta_;
        return wrap<float>(diff, -PI, PI);
    }

    float DMMotorMITController::GetOmegaDelta(float target) const {
        return target - output_shaft_omega_;
    }

    void DMMotorMITController::RegisterErrorCallback(callback_t callback, void* instance) {
        (void)callback;
        (void)instance;
    }

    float DMMotorMITController::GetTheta() const {
        return DMMotor::current_position_;
    }

    float DMMotorMITController::GetOmega() const {
        return DMMotor::current_velocity_;
    }

}  // namespace driver