//
// Created by Administrator on 2026/2/1.
//

#include "chassis.h"

#include "bsp_error_handler.h"
#include "bsp_os.h"

#define M3508_POWER_MODEL {0.000931, 0.000455, 0.000006, 39.151850}

namespace WheelLeg {

    Chassis::Chassis(const chassis_t chassis) {
        // acquired from user
        model_ = chassis.model;

        // data initialization using acquired model
        switch (chassis.model) {
            case CHASSIS_WHEELLEG: {
                motors_ = new driver::MotorCANBase*[TwoWheel::motor_num];
                motors_[TwoWheel::left] = chassis.Motors[TwoWheel::left];
                motors_[TwoWheel::right] = chassis.Motors[TwoWheel::right];

                // 速度初始化
                speeds_ = new float[TwoWheel::motor_num];
                for (int i = 0; i < TwoWheel::motor_num; ++i)
                    speeds_[i] = 0;

                // 轮子数
                wheel_num_ = TwoWheel::motor_num;
                break;
            }
            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
        chassis_offset_ = chassis.offset;

        // 功率限制系统
        control::NewPowerLimit::power_param_t power_model[2] = {M3508_POWER_MODEL,
                                                                M3508_POWER_MODEL};
        power_limit_.enabled = chassis.power_limit_on;
        power_limit_.limiter = new control::NewPowerLimit(power_model);
        driver::DMMotorMITController::RegisterPreOutputCallback(ApplyPowerLimitWrapper, this);

        // 底盘是否有超级电容
        if (chassis.has_super_capacitor) {
            has_super_capacitor_ = true;
            super_capacitor_ = chassis.super_capacitor;
        }
    }

    Chassis::~Chassis() {
        driver::DMMotorMITController::RegisterPreOutputCallback([](void* args) { UNUSED(args); },
                                                                nullptr);
        switch (model_) {
            case CHASSIS_WHEELLEG: {
                motors_[TwoWheel::left] = nullptr;
                motors_[TwoWheel::right] = nullptr;
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

    void Chassis::SetPower(bool enabled, float max_power, float current_power, float buffer_remain,
                           bool enable_supercap) {
        power_limit_.enabled = enabled;

        UNUSED(max_power);
        UNUSED(current_power);
        UNUSED(buffer_remain);
        UNUSED(enable_supercap);

        return;
    }

    void Chassis::UpdatePower(bool enabled, float max_watt, float current_voltage,
                              uint8_t buffer_percent) {
        power_limit_.enabled = enabled;
        power_limit_.buffer_percent = buffer_percent;
        // 双板通信下，云台可以选择不更新电压值，底盘可以使用自己采样的电压值
        if (current_voltage > 1)
            power_limit_.voltage = current_voltage;
        power_limit_.max_watt = max_watt;
        // 这里仅记录信息，实际在在所有电机的PID计算完成、准备发送CAN前，调用ApplyPowerLimit直接设置电机输出，以进行功率限制
    }

    void Chassis::UpdatePowerVoltage(float voltage) {
        power_limit_.voltage = voltage;
    }

    void Chassis::ApplyPowerLimitWrapper(void* args) {
        Chassis* chassis = reinterpret_cast<Chassis*>(args);
        chassis->ApplyPowerLimit();
    }
    void Chassis::ApplyPowerLimit() {
        if (!power_limit_.enabled) {
            return;
        }

        // 电流(A) = 功率(W) / 电压(V)
        float max_current = power_limit_.max_watt / power_limit_.voltage;

        // 根据缓冲区剩余能量，使用线性插值，计算最大电流。
        max_current = max_current *
                      linear_interpolation<int>(20, 80, 80, 150, power_limit_.buffer_percent) / 100;

        // 获取数据
        int16_t turn_current[TwoWheel::motor_num];
        float angular_velocity[TwoWheel::motor_num];
        for (uint8_t i = 0; i < TwoWheel::motor_num; ++i) {
            turn_current[i] = motors_[i]->GetOutput();
            angular_velocity[i] = motors_[i]->GetOmega();
        }

        // 在转矩电流之上应用功率限制
        power_limit_.limiter->LimitPower(turn_current, angular_velocity,
                                         (int16_t)(max_current * 1000));

        // 应用限制后的转矩电流
        for (uint8_t i = 0; i < TwoWheel::motor_num; ++i)
            motors_[i]->SetOutput((int16_t)turn_current[i]);
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
            case CHASSIS_WHEELLEG: {
                motors_[TwoWheel::left] = nullptr;
                motors_[TwoWheel::right] = nullptr;
                break;
            }
            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
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

    JointControl::JointControl(const Joint_t Joint) {
        model_ = Joint.model;

        switch (model_) {
            case CHASSIS_WHEELLEG: {
                motors_ = new driver::DMMotorMITController*[TwoWheel::motor_num];
                motors_[TwoWheel::left] = Joint.JoinMotors[TwoWheel::left];
                motors_[TwoWheel::right] = Joint.JoinMotors[TwoWheel::right];

                Linkage_[TwoWheel::left] = Joint.Linkage[TwoWheel::left];
                Linkage_[TwoWheel::right] = Joint.Linkage[TwoWheel::right];

                // 角度初始化
                theta_ = new float[TwoWheel::motor_num];
                for (int i = 0; i < TwoWheel::motor_num; ++i)
                    theta_[i] = 0;

                // 轮子数
                JoinMotor_num_ = TwoWheel::motor_num;
                break;
            }
            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }

        offset_ = Joint.offset;
    }

    JointControl::~JointControl() {
        driver::DMMotorMITController::RegisterPreOutputCallback([](void* args) { UNUSED(args); },
                                                                nullptr);
        switch (model_) {
            case CHASSIS_WHEELLEG: {
                motors_[JoinMotor::front_left] = nullptr;
                motors_[JoinMotor::front_right] = nullptr;
                motors_[JoinMotor::back_left] = nullptr;
                motors_[JoinMotor::back_right] = nullptr;

                Linkage_[TwoWheel::left] = nullptr;
                Linkage_[TwoWheel::right] = nullptr;

                delete[] motors_;
                motors_ = nullptr;

                delete[] Linkage_;
                Linkage_ = nullptr;

                delete[] theta_;
                theta_ = nullptr;
                break;
            }

            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

    void JointControl::SetPosition(const LegPos_t Left_LegPos, const LegPos_t Right_LegPos) {
        Heartbeat();
        Left_LegPos_ = Left_LegPos;
        Right_LegPos_ = Right_LegPos;

        switch (model_) {
            case CHASSIS_WHEELLEG: {
                kin_ik_result_t result_left = Linkage_[TwoWheel::left]->inverseKinematics(Left_LegPos.FootEnd_X, Left_LegPos.FootEnd_Y);
                theta_[JoinMotor::front_left] = result_left.alpha;
                theta_[JoinMotor::back_left] = result_left.beta;

                kin_ik_result_t result_right = Linkage_[TwoWheel::right]->inverseKinematics(Left_LegPos.FootEnd_X, Left_LegPos.FootEnd_Y);
                theta_[JoinMotor::front_right] = result_right.alpha;
                theta_[JoinMotor::back_right] = result_right.beta;

                break;
            }
            default:
                RM_ASSERT_TRUE(false, "Not Supported Chassis Mode\r\n");
        }
    }

}  // namespace WheelLeg