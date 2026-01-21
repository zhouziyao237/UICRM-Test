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

//
// Created by Administrator on 2025/11/19.
//

#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "bsp_can.h"
#include "connection_driver.h"
#include "main.h"

namespace driver {

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
     * @brief 达妙4310电机控制类
     *
     * @note 使用CAN通信，波特率1Mbps，支持MIT/位置速度/速度三种模式
     *       反馈帧格式统一，控制帧格式随模式变化
     */
    class DMMotor : public ConnectionDriver {
      public:
        /**
         * @brief 构造函数
         *
         * @param can    CAN总线对象
         * @param rx_id  接收ID（MasterID，用于反馈帧）
         * @param tx_id  电机ID（基础ID，用于MIT模式）
         * @param mode   电机模式（MIT/POS_VEL/VEL）
         *
         * @note 实际发送ID根据模式自动计算：
         *       MIT模式：tx_id
         *       位置速度模式：tx_id + 0x100
         *       速度模式：tx_id + 0x200
         */
        DMMotor(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id, dm_motor_mode_t mode);

        /**
         * @brief 更新电机反馈数据（CAN回调函数内部调用）
         *
         * @param data CAN反馈帧数据（8字节）
         *
         * @note 反馈帧格式：[ID & ERR | POS_H | POS_L | VEL_H & VEL_L | T_H & T_L | T_MOS | T_Rotor]
         *       位置：16位，速度：12位，扭矩：12位
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
         * @brief 获取当前错误状态码
         *
         * @return 错误码（0-正常，其他参见说明书故障类型）
         */
        int GetErrorState() const;

        /**
         * @brief 获取电机CAN ID
         */
        int GetMotorID() const;

        /**
         * @brief 发送控制指令到电机（需先调用SetOutput设置参数）
         */
        void TransmitOutput();

        /**
         * @brief 打印当前电机状态（调试用）
         */
        void PrintData();

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

      private:
        bsp::CAN* can_;
        uint16_t rx_id_;          // 接收ID（MasterID）
        uint16_t tx_id_base_;     // 基础ID（MIT模式使用）
        uint16_t tx_id_actual_;   // 实际发送ID（根据模式计算）

        volatile int motor_id_ = 0;     // 电机ID（反馈帧中获取）
        volatile int error_state_ = 0;  // 错误状态码

        volatile dm_motor_mode_t mode_; // 当前控制模式

        // 当前设置值（待发送）
        volatile float set_position_ = 0;
        volatile float set_velocity_ = 0;
        volatile float set_kp_ = 0;
        volatile float set_kd_ = 0;
        volatile float set_torque_ = 0;

        // 反馈原始数据
        volatile int16_t raw_position_ = 0;
        volatile int16_t raw_velocity_ = 0;
        volatile int16_t raw_torque_ = 0;
        volatile int16_t raw_mos_temp_ = 0;
        volatile int16_t raw_motor_temp_ = 0;

        // 解码后的物理量
        volatile float current_position_ = 0;  // 单位：rad
        volatile float current_velocity_ = 0;  // 单位：rad/s
        volatile float current_torque_ = 0;    // 单位：Nm

        // 参数映射范围（与说明书一致）
        constexpr static float KP_MIN = 0;
        constexpr static float KP_MAX = 500;
        constexpr static float KD_MIN = 0;
        constexpr static float KD_MAX = 5;
        constexpr static float POS_MIN = -12.5f;
        constexpr static float POS_MAX = 12.5f;
        constexpr static float VEL_MIN = -45.0f;
        constexpr static float VEL_MAX = 45.0f;
        constexpr static float TORQUE_MIN = -18.0f;
        constexpr static float TORQUE_MAX = 18.0f;
    };

}  // namespace driver

#endif // DM_MOTOR_H