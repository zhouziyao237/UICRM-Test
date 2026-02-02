//
// Created by Administrator on 2026/2/1.
//

#pragma once

#include "DMmotor.h"
#include "MotorCanBase.h"
#include "can_bridge.h"
#include "connection_driver.h"
#include "fivebar.h"
#include "pid.h"
#include "power_limit_new.h"
#include "supercap.h"
#include "utils.h"

#define MAX_WHEEL_NUM 8

namespace WheelLeg {

    /**
     * @brief chassis models
     */
    typedef enum { CHASSIS_WHEELLEG } chassis_model_t;

    /**
     * @brief structure used when chassis instance is initialized
     */
    struct chassis_t {
        driver::MotorCANBase** Motors; /* motor instances of all chassis motors */
        chassis_model_t model;         /* chassis model */
        float offset = 0;
        bool power_limit_on = false;
        bool has_super_capacitor = false;
        driver::SuperCap* super_capacitor = nullptr;
    };

    struct TwoWheel {
        enum { left, right, motor_num };
    };

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
         * @brief 使用外部采集的底盘数据，更新功率控制信息。
         * @note 这里仅记录信息，实际在回调函数中进行功率控制的计算。
         * @param enabled 功率控制开关
         * @param max_watt 最大功率，单位为W（根据buffer动态调整，有时会更大）
         * @param current_voltage 电池电压，单位为V
         * @param buffer_percent 剩余缓冲能量百分比，范围为0~100，50%对应于最大电流的90%
         */
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

      public:
        // 在所有电机的PID计算完成、准备发送CAN前，调用此函数直接设置电机输出，以进行功率限制
        static void ApplyPowerLimitWrapper(void* args);

      private:
        void ApplyPowerLimit();

      private:
        // acquired from user

        driver::MotorCANBase** motors_ = nullptr;
        chassis_model_t model_;

        // pids and current speeds for each motor on the chassis
        float* speeds_ = nullptr;

        uint8_t wheel_num_ = 0;

        struct {
            bool enabled = false;
            uint8_t buffer_percent = 0;  // 0~100
            float max_watt = 0;
            float voltage = 0;

            control::NewPowerLimit* limiter = nullptr;
        } power_limit_;

        float chassis_offset_;

        uint8_t can_bridge_tx_id_ = 0x00;
        volatile float can_bridge_vx_ = 0;
        volatile float can_bridge_vy_ = 0;
        volatile float can_bridge_vt_ = 0;

        bool chassis_enable_ = true;

        bool has_super_capacitor_ = false;
        bool super_capacitor_enable_ = false;
        driver::SuperCap* super_capacitor_ = nullptr;

        float max_motor_speed_ = 2 * PI * 10;

      public:
        // Record for debugging
        float chassis_vx = 0;
        float chassis_vy = 0;
        float chassis_vt = 0;
    };

    struct JoinMotor {
        enum { front_left, front_right, back_left, back_right, Join_motor_num };
    };

    struct JointOffset {
        float front_left = 0;
        float front_right = 0;
        float back_left = 0;
        float back_right = 0;
    };

    struct Joint_t {
        driver::DMMotorMITController** JoinMotors; /* Join motor instances of all chassis motors */
        FiveBarLinkage** Linkage;
        chassis_model_t model; /* chassis model */
        JointOffset offset;
        // bool power_limit_on = false;
        // bool has_super_capacitor = false;
        // driver::SuperCap* super_capacitor = nullptr;
    };

    struct LegPos_t {
        float FootEnd_X = 0;
        float FootEnd_Y = 0;

        float L0 = 0;
        float Theta0 = 0;
    };

    class JointControl : public driver::ConnectionDriver {
    public:
        explicit JointControl(const Joint_t Joint);

        ~JointControl() override;

        void SetPosition(const LegPos_t Left_LegPos, const LegPos_t Right_LegPos);


    protected:
        driver::DMMotorMITController** motors_ = nullptr;
        FiveBarLinkage** Linkage_ = nullptr;
        chassis_model_t model_;

        JointOffset offset_;

        // pids and current speeds for each motor on the chassis
        float* theta_ = nullptr;

        uint8_t JoinMotor_num_ = 0;

        LegPos_t Left_LegPos_;
        LegPos_t Right_LegPos_;
    }

}  // namespace WheelLeg