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

#pragma once

#include "bsp_can.h"
#include "bsp_thread.h"
#include "connection_driver.h"
#include "pid.h"
#include "utils.h"

#define TORQUE_REVERSE_MODE

namespace driver {

    /**
     * @brief 达妙电机状态码（与说明书对应）
     * @note ERR字段包含使能状态和故障状态
     */
    typedef enum {
        DM_STATE_DISABLED = 0x0,  // 失能状态
        DM_STATE_ENABLED = 0x1,   // 使能状态

        // 故障状态（从0x8开始）
        DM_FAULT_OVERVOLT = 0x8,        // 超压
        DM_FAULT_UNDERVOLT = 0x9,       // 欠压
        DM_FAULT_OVERCURRENT = 0xA,     // 过流
        DM_FAULT_MOS_OVERTEMP = 0xB,    // MOS过温
        DM_FAULT_MOTOR_OVERTEMP = 0xC,  // 电机过温
        DM_FAULT_COMM_LOSS = 0xD,       // 通讯丢失
        DM_FAULT_OVERLOAD = 0xE,        // 过载
    } dm_motor_state_t;

    /**
     * @brief 达妙电机防护配置结构体
     */
    struct DMMotorProtectConfig {
        float overcurrent_threshold = 9.8f;      // 过流阈值（A）- 参考说明书建议值
        float overvoltage_threshold = 32.0f;     // 过压阈值（V）- 参考说明书建议值
        float undervoltage_threshold = 15.0f;    // 欠压阈值（V）- 参考说明书建议值
        uint8_t motor_overtemp_threshold = 100;  // 电机过温阈值（°C）- 参考说明书建议值
        uint8_t mos_overtemp_threshold = 120;    // MOS过温阈值（°C）- 参考说明书建议值
        uint16_t overload_duration = 1000;       // 过载持续时间（ms）

        // 回调函数
        void (*fault_callback)(void* instance, dm_motor_state_t fault) = nullptr;
        void* fault_callback_instance = nullptr;
    };

    /**
     * @brief 达妙4310电机操作模式（与说明书对应）
     *
     * @note 电机操作模式决定控制帧格式和使用的ID偏移量
     *
     * @enum MIT      MIT模式（原MIT兼容模式），控制帧包含位置、速度、Kp、Kd、扭矩
     * @enum POS_VEL  位置速度模式（位置串级模式），控制帧包含位置和速度
     * @enum VEL      速度模式，控制帧仅包含速度
     */
    typedef enum {
        MIT = 0,      // MIT模式，ID偏移为0
        POS_VEL = 1,  // 位置速度模式，ID偏移为0x100
        VEL = 2,      // 速度模式，ID偏移为0x200
    } dm_motor_mode_t;

    /**
     * @brief 达妙4310电机控制类（带自动发送线程和防护功能）
     *
     * @note 使用CAN通信，波特率1Mbps，支持MIT/位置速度/速度三种模式
     *       反馈帧格式统一，控制帧格式随模式变化
     *       内置后台线程自动发送控制指令，用户只需设置参数
     *       具备完善的防护功能（过压、欠压、过流、过温、过载等）
     */
    class DMMotor : public ConnectionDriver {
      public:
        /**
         * @brief 软件侧电机控制状态枚举
         */
        enum ControlState {
            DISABLED = 0x00,  // 控制侧失能
            ENABLED = 0x01,   // 控制侧已使能
            WAITING = 0x02,   // 等待运动
            MOVING = 0x03,    // 正在运动
            STABLE = 0x04,    // 稳定状态
            FAULT = 0x05,     // 故障状态
        };

        /**
         * @brief 堵转回调函数类型（与MotorCANBase保持一致）
         */
        typedef void (*callback_t)(void* args);

        /**
         * @brief 构造函数
         *
         * @param can    CAN总线对象
         * @param rx_id  接收ID（MasterID，用于反馈帧）
         * @param tx_id  电机ID（基础ID，用于MIT模式）
         * @param mode   电机模式（MIT/POS_VEL/VEL）
         * @param config 防护配置（可选）
         *
         * @note 实际发送ID根据模式自动计算：
         *       MIT模式：tx_id
         *       位置速度模式：tx_id + 0x100
         *       速度模式：tx_id + 0x200
         *       后台线程自动发送控制指令
         */
        DMMotor(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id, dm_motor_mode_t mode,
                DMMotorProtectConfig config = DMMotorProtectConfig());

        /**
         * @brief 设置控制频率
         *
         * @param freq 控制频率（Hz），默认500Hz
         * @note 必须在创建第一个电机前调用
         */
        static void SetControlFrequency(uint32_t freq);

        /**
         * @brief 更新电机反馈数据（CAN回调函数内部调用）
         *
         * @param data CAN反馈帧数据（8字节）
         *
         * @note 反馈帧格式：[ID & ERR | POS_H | POS_L | VEL_H & VEL_L | T_H & T_L | T_MOS |
         * T_Rotor] 位置：16位，速度：12位，扭矩：12位
         */
        void UpdateData(const uint8_t data[]);

        /**
         * @brief 使能电机（进入使能模式）
         */
        void MotorEnable();

        /**
         * @brief 失能电机（退出使能模式）
         */
        void MotorDisable();

        /**
         * @brief 设置当前位置为零点
         */
        void SetZeroPosition();

        /**
         * @brief 获取电机当前位置（单位：rad）
         */
        float GetPosition() const;

        /**
         * @brief 获取电机当前速度（单位：rad/s）
         */
        float GetVelocity() const;

        /**
         * @brief 获取电机当前扭矩（单位：Nm）
         */
        float GetTorque() const;

        /**
         * @brief 获取电机反馈的硬件状态
         * @return 硬件状态码（参见dm_motor_state_t）
         */
        dm_motor_state_t GetHardwareState() const;

        /**
         * @brief 获取当前控制状态
         */
        ControlState GetControlState() const;

        /**
         * @brief 获取电机CAN ID
         */
        int GetMotorID() const;

        /**
         * @brief 检查是否处于故障状态
         * @return true - 存在故障，false - 正常
         */
        bool IsInFault() const;

        /**
         * @brief 清除故障状态
         * @return true - 清除成功，false - 清除失败
         */
        bool ClearFault();

        /**
         * @brief 打印当前电机状态（调试用）
         */
        void PrintData();

        // === 防护相关函数 ===

        /**
         * @brief 设置防护配置
         * @param config 新的防护配置
         */
        void SetProtectConfig(const DMMotorProtectConfig& config);

        /**
         * @brief 获取当前防护配置
         */
        DMMotorProtectConfig GetProtectConfig() const;

        /**
         * @brief 注册故障回调函数
         * @param callback 回调函数
         * @param instance 回调函数参数
         */
        void RegisterFaultCallback(void (*callback)(void* instance, dm_motor_state_t fault),
                                   void* instance);

        // === 参数设置函数（模式相关）===

        /**
         * @brief 设置MIT模式下的控制参数
         *
         * @param position 目标位置（单位：rad，范围：-12.5~12.5）
         * @param velocity 目标速度（单位：rad/s，范围：-45~45）
         * @param kp       位置比例系数（范围：0~500）
         * @param kd       位置微分系数（范围：0~5）
         * @param torque   转矩前馈（单位：Nm，范围：-18~18）
         */
        void SetMITParams(float position, float velocity, float kp, float kd, float torque);

        /**
         * @brief 设置位置速度模式下的控制参数
         *
         * @param position 目标位置（单位：rad，浮点数，低位在前）
         * @param velocity 目标速度（单位：rad/s，浮点数，梯形最高速度）
         */
        void SetPositionVelocityParams(float position, float velocity);

        /**
         * @brief 设置速度模式下的控制参数
         *
         * @param velocity 目标速度（单位：rad/s，浮点数，低位在前）
         */
        void SetVelocityParams(float velocity);

        /**
         * @brief 设置在执行输出数据前的回调函数，一般用于功率限制
         * @param callback 回调函数
         * @param instance 回调函数的参数
         */
        static void RegisterPreOutputCallback(callback_t callback, void* instance);

        /**
         * @brief 设置在执行输出数据后的回调函数，一般用于垃圾清理等
         * @param callback 回调函数
         * @param instance 回调函数的参数
         */
        static void RegisterPostOutputCallback(callback_t callback, void* instance);

        /**
         * @brief 扭矩输出函数
         *
         * 该函数用于执行扭矩输出相关的操作，具体实现未在当前代码中提供。
         * 函数可能涉及电机控制、力矩调节或其他与扭矩相关的输出功能。
         *
         * @param 无参数
         *
         * @return 无返回值
         */
        virtual void TorqueOutput();

        // === 数据转换工具函数 ===

        /**
         * @brief 浮点数转定点数（用于MIT模式参数编码）
         *
         * @param x     浮点数值
         * @param x_min 最小值
         * @param x_max 最大值
         * @param bits  位数
         * @return 定点数值
         */
        static uint16_t FloatToUint(float x, float x_min, float x_max, int bits);

        /**
         * @brief 定点数转浮点数（用于反馈数据解码）
         */
        static float UintToFloat(int x_int, float x_min, float x_max, int bits);

        // 角度转换常量
        // todo: 多圈绝对值编码器所以实际一圈是1/4的全部总和圈数
        static constexpr float DAMIAO_HALF_RANGE = 12.5 / 4;  // 达妙角度总范围

        // 参数映射范围（与说明书一致）
        constexpr static float KP_MIN = 0;
        constexpr static float KP_MAX = 500;
        constexpr static float KD_MIN = 0;
        constexpr static float KD_MAX = 5;
        constexpr static float POS_MIN = -12.5f;
        constexpr static float POS_MAX = 12.5f;
        constexpr static float VEL_MIN = -45.0f;
        constexpr static float VEL_MAX = 45.0f;
        constexpr static float TORQUE_MIN = -12.0f;
        constexpr static float TORQUE_MAX = 12.0f;

      protected:
        // 反馈原始数据
        volatile int16_t raw_position_ = 0;
        volatile int16_t raw_velocity_ = 0;
        volatile int16_t raw_torque_ = 0;
        volatile int16_t raw_mos_temp_ = 0;
        volatile int16_t raw_motor_temp_ = 0;

        // 解码后的物理量
        volatile float absolute_position_ = 0;  // 单位：rad
        volatile float current_position_ = 0;   // 单位：rad
        volatile float current_velocity_ = 0;   // 单位：rad/s
        volatile float current_torque_ = 0;     // 单位：Nm

        volatile int motor_id_ = 0;                               // 电机ID（反馈帧中获取）
        volatile dm_motor_state_t hw_state_ = DM_STATE_DISABLED;  // 硬件反馈状态
        volatile ControlState ctrl_state_ = DISABLED;             // 控制侧状态

        bool MITController = false;
        bool SETMode = false;

        static callback_t pre_output_callback_;
        static void* pre_output_callback_instance_;
        static callback_t post_output_callback_;
        static void* post_output_callback_instance_;

      private:
        bsp::CAN* can_;
        uint16_t rx_id_;         // 接收ID（MasterID）
        uint16_t tx_id_base_;    // 基础ID（MIT模式使用）
        uint16_t tx_id_actual_;  // 实际发送ID（根据模式计算）
        dm_motor_mode_t mode_;   // 当前控制模式
        uint32_t comm_loss_time = 0;

        // 当前设置值（待发送）
        float set_position_ = 0;
        float set_velocity_ = 0;
        float set_kp_ = 0;
        float set_kd_ = 0;
        float set_torque_ = 0;

        // 防护相关
        DMMotorProtectConfig protect_config_;                 // 防护配置
        uint32_t overload_start_time_ = 0;                    // 过载开始时间
        bool fault_latched_ = false;                          // 故障锁定标志
        dm_motor_state_t latched_fault_ = DM_STATE_DISABLED;  // 锁定的故障类型

        // === 自动发送线程相关 ===

        /**
         * @brief 打包控制数据（根据模式）
         * @param data 输出的8字节CAN数据
         */
        void PackControlData(uint8_t data[8]);

        /**
         * @brief 执行防护检测
         * @return true - 存在故障，false - 正常
         */
        bool CheckProtection();

        /**
         * @brief 处理故障
         * @param fault 故障类型
         */
        void HandleFault(dm_motor_state_t fault);

        // 静态成员（所有电机共享的线程管理）
        static bool is_init_;
        static bsp::Thread* control_thread_;
        static constexpr osThreadAttr_t thread_attr_ = {.name = "DMMotorCtrl",
                                                        .attr_bits = osThreadDetached,
                                                        .cb_mem = nullptr,
                                                        .cb_size = 0,
                                                        .stack_mem = nullptr,
                                                        .stack_size = 256 * 4,
                                                        .priority = (osPriority_t)osPriorityHigh,
                                                        .tz_module = 0,
                                                        .reserved = 0};

        // 控制频率相关
        static uint32_t control_freq_;
        static uint32_t control_interval_;  // ms

        // 电机实例管理（按CAN线路和ID分组）
        static constexpr uint8_t MAX_GROUPS = 10;
        static constexpr uint8_t MAX_MOTORS_PER_GROUP = 1;  // 达妙电机每个ID独立

        struct MotorGroup {
            bsp::CAN* can;   // CAN总线
            uint16_t tx_id;  // 发送ID
            uint16_t rx_id;  // 接收ID
            uint8_t motor_data_[8];
            DMMotor* motor;  // 电机实例
            bool active;     // 是否激活
        };

        static MotorGroup motor_groups_[MAX_GROUPS];
        static uint8_t group_count_;

        // 后台线程函数
        static void ControlThread(void* args);

        // 向控制线程注册电机
        void RegisterToControlThread();
    };

    /**
     * @brief 达妙电机MIT模式控制器（仿DJI电机控制架构）
     *
     * @note 继承自DMMotor类，专门用于MIT模式的纯力矩控制
     *       支持位置环、速度环、力矩环的级联控制
     *       使用父类已有的控制线程，无需创建新线程
     */
    class DMMotorMITController : public DMMotor {
      public:
        /**
         * @brief 控制模式枚举（与MotorCANBase保持一致）
         */
        enum ControlMode {
            NONE = 0x00,                   // 无控制
            CURRENT = 0x01,                // 电流控制（未使用，保持兼容）
            ANGLE_LOOP_CONTROL = 0x02,     // 角度环控制（与DJI保持一致）
            SPEED_LOOP_CONTROL = 0x04,     // 速度环控制（与DJI保持一致）
            REVERSE_MOTOR_OPERATE = 0x40,  // 反转电机方向（与DJI保持一致）
            MOTOR_TORQUE_REVERSE = 0x70,   // 扭矩输出电机反方向
            ABSOLUTE = 0x80,               // 绝对位置模式（与DJI保持一致）
        };

        /**
         * @brief 堵转回调函数类型（与MotorCANBase保持一致）
         */
        typedef void (*callback_t)(void* instance);

        /**
         * @brief 构造函数
         *
         * @param can    CAN总线对象
         * @param rx_id  接收ID（MasterID）
         * @param tx_id  电机ID（基础ID）
         * @param config 防护配置（可选）
         */
        DMMotorMITController(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id,
                             DMMotorProtectConfig config = DMMotorProtectConfig());

        void GetInitEncoder();

        /**
         * @brief 设置控制模式
         *
         * @param mode 控制模式（位组合，支持多种模式叠加）
         */
        void SetMode(uint8_t mode);

        /**
         * @brief 更新数据
         *
         * 该函数用于更新内部数据缓冲区，使用提供的字节数组替换当前数据
         *
         * @param data[]  原始数据
         * @return 无返回值
         */
        virtual void UpdateData(const uint8_t data[]);

        /**
         * @brief 设置目标值
         *
         * @param target 目标值（单位取决于控制模式）
         * @param override 是否强制覆盖当前目标（默认true）
         *
         * @note 在角度控制模式下，target为角度[rad]
         *       在速度控制模式下，target为角速度[rad/s]
         *       在力矩控制模式下，通过SetOutput()直接设置力矩
         */
        void SetTarget(float target, bool override = true);

        /**
         * @brief 获取当前目标值
         */
        float GetTarget() const;

        /**
         * @brief 计算控制输出（必须在主循环中定期调用）
         *
         * @note 此函数计算PID输出并设置电机控制参数
         *       需要外部定期调用（建议与控制线程同步或略低于控制频率）
         */
        void TorqueOutput();

        /**
         * @brief 使电机停止在当前位置
         *
         * @param override 是否强制保持（默认true）
         */
        void Hold(bool override = true);

        /**
         * @brief 检查电机是否处于保持状态
         */
        bool IsHolding() const;

        /**
         * @brief 重新初始化PID参数
         *
         * @param pid_init PID初始化参数
         * @param mode 控制模式（角度环或速度环）
         */
        void ReInitPID(control::ConstrainedPID::PID_Init_t pid_init, uint8_t mode);

        /**
         * @brief 获取PID状态
         *
         * @param mode 控制模式（角度环或速度环）
         */
        control::ConstrainedPID::PID_State_t GetPIDState(uint8_t mode) const;

        /**
         * @brief 设置角度环接近阈值（与MotorCANBase保持一致）
         *
         * @param proximity_in  进入保持状态的角度阈值[rad]
         * @param proximity_out 退出保持状态的角度阈值[rad]
         *
         * @note proximity_out应大于proximity_in
         */
        void SetProximityThreshold(float proximity_in, float proximity_out);

        /**
         * @brief 设置力矩前馈偏移（对应MotorCANBase的SetOutput）
         *
         * @param torque 目标力矩[Nm]
         */
        void SetOutput(float torque);

        /**
         * @brief 获取当前设置的力矩值
         */
        float GetOutput() const;

        /**
         * @brief 设置速度前馈偏移
         *
         * @param offset 速度偏移量[rad/s]
         */
        void SetSpeedOffset(float offset);

        /**
         * @brief 使能控制器
         * @note 会同时使能电机
         */
        void Enable();

        /**
         * @brief 失能控制器
         * @note 会失能电机并清零力矩
         */
        void Disable();

        /**
         * @brief 检查控制器是否使能
         */
        bool IsEnable() const;

        /**
         * @brief 获取输出轴角度
         */
        float GetOutputShaftTheta() const;

        /**
         * @brief 获取输出轴角速度
         */
        float GetOutputShaftOmega() const;

        /**
         * @brief 获取绝对位置
         */
        float GetAbsolutePosition() const;

        /**
         * @brief 设置减速比
         *
         * @param ratio 减速比
         */
        void SetTransmissionRatio(float ratio);

        /**
         * @brief 获取与目标的角度差
         *
         * @param target 目标角度[rad]
         * @return 角度差[rad]，范围[-PI, PI]
         */
        float GetThetaDelta(float target) const;

        /**
         * @brief 获取与目标的角速度差
         *
         * @param target 目标角速度[rad/s]
         * @return 角速度差[rad/s]
         */
        float GetOmegaDelta(float target) const;

        /**
         * @brief 设置堵转回调函数
         * @param callback 堵转回调函数
         * @param instance 堵转回调函数的参数
         */
        void RegisterErrorCallback(callback_t callback, void* instance);

        /**
         * @brief 处理堵转回调函数
         * @param instance 关联的电机实例
         * @param type pid的故障类型
         */
        static void ErrorCallbackWrapper(void* instance,
                                         control::ConstrainedPID::PID_ErrorHandler_t type);

        /**
         * @brief 获取电机编码器原始角度
         */
        float GetTheta() const;

        /**
         * @brief 获取电机编码器原始角速度
         */
        float GetOmega() const;

        bool is_init = false;

      protected:
        uint8_t mode_;  // 当前控制模式
        float target_;  // 当前目标值
        float output_;  // 当前输出力矩值
        bool holding_;  // 保持状态标志

        // PID控制器（与MotorCANBase使用相同的类）
        control::ConstrainedPID omega_pid_;  // 速度环PID（对应MotorCANBase的omega_pid_）
        control::ConstrainedPID theta_pid_;  // 角度环PID（对应MotorCANBase的theta_pid_）

        // angle control
        volatile float align_angle_ = 0;  /* 对齐角度，开机时的角度，单位为[rad] */
        volatile float motor_angle_ = 0;  /* 当前电机相比于开机的角度的旋转的角度，单位为[rad] */
        volatile float offset_angle_ = 0; /* cumulative offset angle of motor shaft, range between
                                                    [0, 2PI] in [rad] */
        volatile float servo_angle_ = 0;  /* 电机输出轴的角度，单位为[rad]，范围为[0, 2PI] */
        volatile float cumulated_angle_ = 0; /* 累积角度，单位为[rad] */

      private:
        // 前馈偏移
        float speed_offset_;  // 前馈中使用，在角度环输出的速度上加上一个偏移量

        callback_t error_callback_ = [](void* instance) { UNUSED(instance); };
        void* error_callback_instance_ = nullptr;

        float CONTROL_SCALE = 100;  // (todo: 输出范围缩放 100)

        // 位置控制参数（与MotorCANBase保持一致）
        float proximity_in_;   // 进入保持状态阈值[rad]
        float proximity_out_;  // 退出保持状态阈值[rad]

        // 减速比相关（与MotorCANBase保持一致）
        float transmission_ratio_;                // 减速比
        float power_on_angle_;                    // 上电角度
        float relative_angle_;                    // 相对角度
        float cumulated_turns_;                   // 累计圈数
        float output_shaft_theta_;                // 输出轴角度
        float output_shaft_omega_;                // 输出轴角速度
        FloatEdgeDetector* inner_wrap_detector_;  // 内部边界检测器
        FloatEdgeDetector* outer_wrap_detector_;  // 外部边界检测器
    };

    /**
     * @brief DM 6006电机的标准类
     */
    /**
     * @brief DM 6006 motor class
     */
    class Motor6006 : public DMMotorMITController {
      public:
        /* constructor wrapper over MotorCANBase */
        Motor6006(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id);
        /* implements data update callback */
        void UpdateData(const uint8_t data[]) override final;
    };

}  // namespace driver
