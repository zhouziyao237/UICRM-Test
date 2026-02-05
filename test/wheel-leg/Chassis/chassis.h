#pragma once

#include "DMmotor.h"
#include "MotorCanBase.h"
#include "bsp_math.h"
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
     * @brief 错误码枚举
     */
    enum class ErrorCode {
        SUCCESS = 0,
        MOTOR_OFFLINE,
        INVALID_PARAMETER,
        INITIALIZATION_FAILED,
        UNSUPPORTED_MODE
    };

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
        explicit Chassis(const chassis_t chassis);
        ~Chassis() override;

        void SetPower(bool enabled, float max_power, float current_power, float buffer_remain,
                      bool enable_supercap = false);
        void UpdatePower(bool enabled, float max_watt, float current_voltage,
                         uint8_t buffer_percent);
        void UpdatePowerVoltage(float voltage);
        void Update();
        void Enable();
        void Disable();
        void SetMaxMotorSpeed(float max_speed);

      public:
        static void ApplyPowerLimitWrapper(void* args);

      private:
        void ApplyPowerLimit();

      private:
        driver::MotorCANBase** motors_ = nullptr;
        chassis_model_t model_;
        float* speeds_ = nullptr;
        uint8_t wheel_num_ = 0;

        struct {
            bool enabled = false;
            uint8_t buffer_percent = 0;
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
        float chassis_vx = 0;
        float chassis_vy = 0;
        float chassis_vt = 0;
    };

    struct JoinMotor {
        enum { front_left, front_right, back_left, back_right, Join_motor_num };
    };

    struct TwoLink {
        enum { left, right, Link_num };
    };

    struct JointMotorData_t {
        float offset = 0;
        float Max = 0;
    };

    class JointMirror_t {
    public:
        JointMirror_t() : left_enabled_(false), right_enabled_(false) {}
        JointMirror_t(bool left, bool right) : left_enabled_(left), right_enabled_(right) {}

        void SetLeft(bool enabled) { left_enabled_ = enabled; }
        void SetRight(bool enabled) { right_enabled_ = enabled; }

        bool GetLeft() const { return left_enabled_; }
        bool GetRight() const { return right_enabled_; }

        void Reset() {
            left_enabled_ = false;
            right_enabled_ = false;
        }

        bool IsAnyEnabled() const {
            return left_enabled_ || right_enabled_;
        }

    private:
        bool left_enabled_ : 1;
        bool right_enabled_ : 1;
    };

    struct ThetaLimit {
        float THETA1_ANGLE_MAX = 2 / PI;
        float THETA1_ANGLE_MIN = 0;
        float THETA2_ANGLE_MAX = PI;
        float THETA2_ANGLE_MIN = 2 / PI;
    };

    struct Joint_t {
        driver::DMMotorMITController** JoinMotors;
        FiveBarLinkage** Linkage;
        JointMotorData_t** data;
        chassis_model_t model;
        JointMirror_t JointMirror;
    };

    struct LegPos_t {
        float FootEnd_X = 0;
        float FootEnd_Y = 0;
        float L0 = 0;
        float Theta0 = 0;
    };

    class JointControl : public driver::ConnectionDriver {
      public:
        explicit JointControl(const Joint_t& Joint);
        ~JointControl() override;

        void Enable();
        void Disable();
        ErrorCode Update(); // 修改为返回 ErrorCode

        std::pair<kin_ik_result_t, ErrorCode> ComputeLegIK(const LegPos_t& leg_pos, const int link); // 修改为返回 ErrorCode

        void ChaekMirror_X(LegPos_t*& LegPos, const uint16_t link) const;

        void CalculatePosture(LegPos_t**& LegPos, const uint16_t link) const;
        void CalculatePosition(LegPos_t**& LegPos, const uint16_t link) const;

        void UpdatePosition(LegPos_t* Left_LegPos, LegPos_t* Right_LegPos);
        void UpdatePosture(LegPos_t* Left_LegPos, LegPos_t* Right_LegPos);
        void UpdateOffset(JointMotorData_t** data, int i);
        void GetLegPos(LegPos_t& LegPos, const uint16_t link);

      protected:
        driver::DMMotorMITController** motors_ = nullptr;
        FiveBarLinkage** Linkage_ = nullptr;
        JointMotorData_t** data_ = nullptr;
        LegPos_t** LegPos_ = nullptr;
        chassis_model_t model_;
        JointMirror_t JointMirror_;

        ThetaLimit ThetaLimit_;

        float Stance_Position[TwoLink::Link_num][2];

        float* Joint_lower_limit_ = nullptr;
        float* Joint_upper_limit_ = nullptr;

        float* theta_ = nullptr;

        uint8_t JoinMotor_num_ = 0;

        bool Joint_enable_ = true;
    };

}  // namespace WheelLeg
