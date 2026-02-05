//
// Created by Administrator on 2026/2/1.
//

#include "chassis.h"

#include "bsp_error_handler.h"
#include "bsp_math.h"
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

    // 修改构造函数参数为常量引用传递
    JointControl::JointControl(const Joint_t& Joint) {
        model_ = Joint.model;
        data_ = Joint.data;
        JointMirror_ = Joint.JointMirror;

        switch (model_) {
            case CHASSIS_WHEELLEG: {
                motors_ = new driver::DMMotorMITController*[JoinMotor::Join_motor_num];

                theta_ = new float[JoinMotor::Join_motor_num];
                Joint_lower_limit_ = new float[JoinMotor::Join_motor_num];
                Joint_upper_limit_ = new float[JoinMotor::Join_motor_num];

                Linkage_ = new FiveBarLinkage*[TwoLink::Link_num];
                JoinMotor_num_ = JoinMotor::Join_motor_num;

                Linkage_[TwoLink::left] = Joint.Linkage[TwoLink::left];
                Linkage_[TwoLink::right] = Joint.Linkage[TwoLink::right];

                // 赋值时分别设置两个元素
                LegPos_ = new LegPos_t*[TwoLink::Link_num];
                for (int i = 0; i < TwoLink::Link_num; i++) {
                    if ((i == TwoLink::left && JointMirror_.GetLeft()) ||
                        (i == TwoLink::right && JointMirror_.GetRight())) {
                        Stance_Position[i][0] = -Linkage_[i]->model_.L5 / 2;
                    } else {
                        Stance_Position[i][0] = Linkage_[i]->model_.L5 / 2;
                    }
                    Stance_Position[i][1] = 0.0f;
                }

                // 角度初始化
                for (int i = 0; i < JoinMotor::Join_motor_num; ++i) {
                    if (data_[i] || motors_[i]) {
                        theta_[i] = data_[i]->offset;
                        motors_[i] = Joint.JoinMotors[i];
                    } else {
                        RM_ASSERT_TRUE(false, "[ERROR] Not Motor Config Data!\r\n");
                    }
                }

                for (int i = 0; i < JoinMotor_num_; i++) {
                    Joint_lower_limit_[i] = wrap<float>(
                        static_cast<float>(data_[i]->offset - data_[i]->Max), 0,
                        2 * static_cast<float>(motors_[i]->DAMIAO_HALF_RANGE));  // 显式类型转换
                    Joint_upper_limit_[i] = wrap<float>(
                        static_cast<float>(data_[i]->offset + data_[i]->Max), 0,
                        2 * static_cast<float>(motors_[i]->DAMIAO_HALF_RANGE));  // 显式类型转换
                }

                break;
            }
            default:
                RM_ASSERT_TRUE(false, "[ERROR] Not Supported Chassis Mode\r\n");
        }
    }

    JointControl::~JointControl() {
        driver::DMMotor::RegisterPreOutputCallback([](void* args) { UNUSED(args); }, nullptr);
        switch (model_) {
            case CHASSIS_WHEELLEG: {
                motors_[JoinMotor::front_left] = nullptr;
                motors_[JoinMotor::front_right] = nullptr;
                motors_[JoinMotor::back_left] = nullptr;
                motors_[JoinMotor::back_right] = nullptr;

                Linkage_[TwoLink::left] = nullptr;
                Linkage_[TwoLink::right] = nullptr;

                LegPos_[TwoLink::left] = nullptr;
                LegPos_[TwoLink::right] = nullptr;

                Joint_lower_limit_ = nullptr;
                Joint_upper_limit_ = nullptr;

                delete[] motors_;
                motors_ = nullptr;

                delete[] Linkage_;
                Linkage_ = nullptr;

                delete[] LegPos_;
                LegPos_ = nullptr;

                delete[] data_;
                data_ = nullptr;

                delete[] theta_;
                theta_ = nullptr;

                break;
            }

            default:
                RM_ASSERT_TRUE(false, "[ERROR] Not Supported Chassis Mode\r\n");
        }
    }

    void JointControl::Enable() {
        Joint_enable_ = true;

        for (int i = 0; i < JoinMotor_num_; i++) {
            if (!motors_[i]->IsEnable()) {
                motors_[i]->Enable();
            }
        }
    }

    void JointControl::Disable() {
        Joint_enable_ = false;

        if (!Joint_enable_) {
            for (int i = 0; i < JoinMotor_num_; i++) {
                motors_[i]->Disable();
            }
        }
    }

    // 修改 Update 函数返回 ErrorCode
    ErrorCode JointControl::Update() {
        // bool need_shutdown = !IsOnline();
        // for (int i = 0; i < JoinMotor_num_; i++) {
        //     if (!motors_[i]->IsOnline()) {
        //         need_shutdown = true;
        //         print("[ERROR] Checked motor offline, Kill Chassis!\r\n");
        //         return ErrorCode::MOTOR_OFFLINE;
        //     }
        // }
        //
        // if (need_shutdown) {
        //     Disable();
        //     return ErrorCode::INITIALIZATION_FAILED;
        // }

        switch (model_) {
            case CHASSIS_WHEELLEG: {
                if (Joint_enable_) {
                    motors_[JoinMotor::front_left]->SetTarget(theta_[JoinMotor::front_left]);
                    motors_[JoinMotor::front_right]->SetTarget(theta_[JoinMotor::front_right]);
                    motors_[JoinMotor::back_left]->SetTarget(theta_[JoinMotor::back_left]);
                    motors_[JoinMotor::back_right]->SetTarget(theta_[JoinMotor::back_right]);
                }
                break;
            }
            default:
                RM_ASSERT_TRUE(false, "[ERROR] Unsupported chassis mode\r\n");
                // return ErrorCode::UNSUPPORTED_MODE;
        }

        return ErrorCode::SUCCESS;
    }

    // 提取逆运动学计算逻辑
    std::pair<kin_ik_result_t, ErrorCode> JointControl::ComputeLegIK(const LegPos_t& leg_pos,
                                                                     const int link) {
        if (link >= TwoLink::Link_num || !Linkage_[link]) {
            return {{}, ErrorCode::INVALID_PARAMETER};
        }

        kin_ik_result_t result;
        try {
            result = Linkage_[link]->inverseKinematics(leg_pos.FootEnd_X, leg_pos.FootEnd_Y);
        } catch (...) {
            return {{}, ErrorCode::INITIALIZATION_FAILED};
        }

        bool is_theta1_valid = (float(result.angles.theta1) > ThetaLimit_.THETA1_ANGLE_MIN &&
                                float(result.angles.theta1) < ThetaLimit_.THETA1_ANGLE_MAX);
        bool is_theta2_valid = (float(result.angles.theta2) > ThetaLimit_.THETA2_ANGLE_MIN &&
                                float(result.angles.theta2) < ThetaLimit_.THETA2_ANGLE_MAX);

        bool is_valid = (is_theta1_valid && is_theta2_valid);
        return {result, is_valid ? ErrorCode::SUCCESS : ErrorCode::INVALID_PARAMETER};
    }

    void JointControl::ChaekMirror_X(LegPos_t*& LegPos, const uint16_t link) const {
        // 根据镜像标志处理 X 坐标
        if ((link == TwoLink::left && JointMirror_.GetLeft()) ||
            (link == TwoLink::right && JointMirror_.GetRight())) {
            // 修正：直接访问指针指向的对象，而不是数组索引
            LegPos->FootEnd_X = -LegPos->FootEnd_X;
            LegPos->Theta0 = wrap<float>(PI - LegPos->Theta0, 0, PI);
        }
    }

    void JointControl::CalculatePosture(LegPos_t**& LegPos, const uint16_t link) const {
        if (!LegPos || link >= TwoLink::Link_num) {
            return;
        }

        float& L0 = LegPos[link]->L0;
        float x_diff = LegPos[link]->FootEnd_X - Stance_Position[link][0];
        float y_diff = LegPos[link]->FootEnd_Y - Stance_Position[link][1];

        L0 = MathSqrtf(MathPow(x_diff, 2) + MathPow(y_diff, 2));

        if (L0 == 0.0f) {
            LegPos[link]->Theta0 = 0.0f;
            return;
        }

        float ratio = Stance_Position[link][0] / L0;
        constexpr float MIN_RATIO = -1.0f;
        constexpr float MAX_RATIO = 1.0f;
        ratio = MathClamp(ratio, MIN_RATIO, MAX_RATIO);

        if (ratio > MAX_RATIO - FLT_EPSILON) {
            ratio = MAX_RATIO;
        } else if (ratio < MIN_RATIO + FLT_EPSILON) {
            ratio = MIN_RATIO;
        }

        LegPos[link]->Theta0 = MathAcos(ratio);
    }

    void JointControl::CalculatePosition(LegPos_t**& LegPos, const uint16_t link) const {
        if (!LegPos || link >= TwoLink::Link_num) {
            return;
        }

        const float& L0 = LegPos[link]->L0;
        LegPos[link]->FootEnd_X = Stance_Position[link][0] + (L0 * MathCos(LegPos[link]->Theta0));
        LegPos[link]->FootEnd_Y = L0 * MathSin(LegPos[link]->Theta0);
    }

    void JointControl::UpdatePosition(LegPos_t* Left_LegPos, LegPos_t* Right_LegPos) {
        Heartbeat();

        ChaekMirror_X(Left_LegPos, TwoLink::left);
        ChaekMirror_X(Right_LegPos, TwoLink::right);

        LegPos_[TwoLink::left] = Left_LegPos;
        LegPos_[TwoLink::right] = Right_LegPos;

        switch (model_) {
            case CHASSIS_WHEELLEG: {
                CalculatePosture(LegPos_, TwoLink::left);
                CalculatePosture(LegPos_, TwoLink::right);

                // 计算左右腿逆运动学结果
                auto [left_result, left_valid] =
                    ComputeLegIK(*LegPos_[TwoLink::left], TwoLink::left);
                auto [right_result, right_valid] =
                    ComputeLegIK(*LegPos_[TwoLink::right], TwoLink::right);

                // 更新关节角度
                // if (left_valid == ErrorCode::SUCCESS && right_valid == ErrorCode::SUCCESS) {
                    theta_[JoinMotor::front_left] =
                        left_result.alpha - data_[JoinMotor::front_left]->offset;
                    theta_[JoinMotor::back_left] =
                        left_result.beta - data_[JoinMotor::back_left]->offset;

                    theta_[JoinMotor::front_right] =
                        right_result.beta - data_[JoinMotor::front_right]->offset;
                    theta_[JoinMotor::back_right] =
                        right_result.alpha - data_[JoinMotor::back_right]->offset;

                    for (int i = 0; i < JoinMotor_num_; i++) {
                        theta_[i] =
                            clip<float>(theta_[i], Joint_lower_limit_[i], Joint_upper_limit_[i]);
                    }
                // }

                break;
            }
            default:
                RM_ASSERT_TRUE(false, "[ERROR] Unsupported chassis mode\r\n");
        }
    }

    void JointControl::UpdatePosture(LegPos_t* Left_LegPos, LegPos_t* Right_LegPos) {
        Heartbeat();

        ChaekMirror_X(Left_LegPos, TwoLink::left);
        ChaekMirror_X(Right_LegPos, TwoLink::right);

        LegPos_[TwoLink::left] = Left_LegPos;
        LegPos_[TwoLink::right] = Right_LegPos;

        switch (model_) {
            case CHASSIS_WHEELLEG: {
                CalculatePosition(LegPos_, TwoLink::left);
                CalculatePosition(LegPos_, TwoLink::right);

                auto [left_result, left_valid] =
                    ComputeLegIK(*LegPos_[TwoLink::left], TwoLink::left);
                auto [right_result, right_valid] =
                    ComputeLegIK(*LegPos_[TwoLink::right], TwoLink::right);

                if (left_valid == ErrorCode::SUCCESS && right_valid == ErrorCode::SUCCESS) {
                    theta_[JoinMotor::front_left] =
                        left_result.alpha - data_[JoinMotor::front_left]->offset;
                    theta_[JoinMotor::back_left] =
                        left_result.beta - data_[JoinMotor::back_left]->offset;

                    theta_[JoinMotor::front_right] =
                        right_result.beta - data_[JoinMotor::front_right]->offset;
                    theta_[JoinMotor::back_right] =
                        right_result.alpha - data_[JoinMotor::back_right]->offset;

                    for (int i = 0; i < JoinMotor_num_; i++) {
                        theta_[i] =
                            clip<float>(theta_[i], Joint_lower_limit_[i], Joint_upper_limit_[i]);
                    }
                }

                break;
            }
            default:
                RM_ASSERT_TRUE(false, "[ERROR] Unsupported chassis mode\r\n");
        }
    }

    void JointControl::UpdateOffset(JointMotorData_t** data, int i) {
        if (!data)
            return;
        data_ = data;

        Joint_lower_limit_[i] =
            wrap<float>(static_cast<float>(data_[i]->offset - data_[i]->Max), 0,
                        2 * static_cast<float>(motors_[i]->DAMIAO_HALF_RANGE));  // 显式类型转换
        Joint_upper_limit_[i] =
            wrap<float>(static_cast<float>(data_[i]->offset + data_[i]->Max), 0,
                        2 * static_cast<float>(motors_[i]->DAMIAO_HALF_RANGE));  // 显式类型转换
    }

    void JointControl::GetLegPos(LegPos_t& LegPos, const uint16_t link) {
        LegPos = *LegPos_[link];
    }

}  // namespace WheelLeg