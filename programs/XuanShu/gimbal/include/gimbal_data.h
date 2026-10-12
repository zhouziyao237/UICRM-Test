#pragma once

#include <cstdint>

#include "pid.h"
#include "remote_input.h"
#include "utils.h"

namespace xuanshu::gimbal_config {

// -----------------------------------------------------------------------------
// Device mapping - XuanShu test gimbal on DJI Board Type-C
// -----------------------------------------------------------------------------
inline constexpr uint16_t kYawMotorRxId = 0x205U;
inline constexpr uint16_t kPitchMotorRxId = 0x206U;
inline constexpr float kMotorTransmissionRatio = 1.0f;

// If false, the first valid motor feedback pose is used as the temporary
// mechanical center. This is the safe default for a new test rig.
inline constexpr bool kUseConfiguredEncoderOffsets = true;
inline constexpr float kPitchOffsetRad = 2.6538f;  // fill after calibration
inline constexpr float kYawOffsetRad =  4.7286f;    // fill after calibration

// Mechanical range. Tune these before allowing large movements.
// Yaw total travel = pi rad:
// [-pi/2, +pi/2]
inline constexpr float kYawMaxRad = 1.57079632679f;
// Pitch:
// [-pi/4, +pi/4]
inline constexpr float kPitchMaxRad = 0.78539816339f;
// Yaw has physical stops; it is not continuous.
inline constexpr bool kYawContinuous = false;
inline constexpr bool kPitchInverted = false;
inline constexpr bool kYawInverted = false;

// Small world-angle deadband used by control::Gimbal::UpdateIMU().
inline constexpr float kPitchEpositionRad = 0.0f;
inline constexpr float kYawEpositionRad = 0.0f;

// -----------------------------------------------------------------------------
// RC mapping
// Right switch: UP=KILL, MID=MANUAL, DOWN=AUTOAIM.
// Manual control defaults to right stick; mouse overrides the stick.
// -----------------------------------------------------------------------------
inline constexpr remote::SwitchSide kModeSwitch = remote::SwitchSide::RIGHT;
inline constexpr remote::switch_t kKillPosition = remote::UP;
inline constexpr remote::switch_t kManualPosition = remote::MID;
inline constexpr remote::switch_t kAutoAimPosition = remote::DOWN;

inline constexpr remote::Channel kPitchChannel = remote::Channel::RIGHT_Y;
inline constexpr remote::Channel kYawChannel = remote::Channel::RIGHT_X;
inline constexpr float kPitchChannelSign = -1.0f;
inline constexpr float kYawChannelSign = -1.0f;
inline constexpr float kStickDeadband = 0.06f;
inline constexpr float kMouseDeadband = 0.015f;

// Commanded angular velocity in MANUAL mode.
inline constexpr float kPitchStickRateRadS = 2.4f;
inline constexpr float kYawStickRateRadS = 3.2f;
inline constexpr float kPitchMouseRateRadS = 1.8f;
inline constexpr float kYawMouseRateRadS = 3.0f;
inline constexpr float kMousePitchSign = -1.0f;
inline constexpr float kMouseYawSign = 1.0f;

// -----------------------------------------------------------------------------
// IMU mapping - BMI088 gyro + accelerometer only; no magnetometer.
// Tune here after checking the real C-board installation.
// The AHRS output is indexed [0..2]. Keep all physical axis/sign conversion
// here so gimbal_task.cpp stays unchanged during calibration.
// -----------------------------------------------------------------------------
inline constexpr uint8_t kImuYawAxis = 0U;
inline constexpr uint8_t kImuPitchAxis = 1U;
inline constexpr uint8_t kImuRollAxis = 2U;
inline constexpr float kImuYawSign = 1.0f;
inline constexpr float kImuPitchSign = -1.0f;
inline constexpr float kImuRollSign = 1.0f;
inline constexpr float kImuHeaterTargetC = 45.0f;
inline constexpr uint32_t kImuOfflineTimeoutMs = 100U;

// -----------------------------------------------------------------------------
// Auto-aim mapping
// Existing UICRM Host TARGET_ANGLE is reused. For the current Host API, target
// validity is determined by Host online + accuracy threshold. The vision side
// should send TARGET_ANGLE with accuracy=0 when no target is available.
// -----------------------------------------------------------------------------
inline constexpr float kVisionPitchSign = 1.0f;
inline constexpr float kVisionYawSign = 1.0f;
inline constexpr uint8_t kVisionMinAccuracy = 1U;
inline constexpr uint32_t kMiniPCOfflineTimeoutMs = 120U;

// true: TARGET_ANGLE is a desired world/IMU angle (Atlas/Minnie style).
// false: TARGET_ANGLE is treated as an angular error relative to current IMU.
inline constexpr bool kVisionTargetIsAbsoluteWorld = true;

// -----------------------------------------------------------------------------
// Task timing
// -----------------------------------------------------------------------------
inline constexpr uint32_t kRcPeriodMs = 1U;
inline constexpr uint32_t kSysPeriodMs = 5U;
inline constexpr uint32_t kGimbalPeriodMs = 2U;      // 500 Hz command update
inline constexpr uint32_t kMiniPCPeriodMs = 5U;     // 200 Hz service
inline constexpr uint32_t kDebugPeriodMs = 100U;    // 10 Hz console
inline constexpr uint32_t kDbusOfflineTimeoutMs = 80U;
inline constexpr uint32_t kMotorStartupTimeoutMs = 1000U;

// -----------------------------------------------------------------------------
// M6020 PID - initial conservative test values. Tune here only.
// -----------------------------------------------------------------------------
inline control::ConstrainedPID::PID_Init_t PitchAnglePid() {
    return {
        .kp = 40.0f,
        .ki = 0.0f,
        .kd = 1200.0f,
        .max_out = 5.0f * PI,
        .max_iout = 0.0f,
        .deadband = 0.0f,
        .A = 0.0f,
        .B = 0.0f,
        .output_filtering_coefficient = 0.10f,
        .derivative_filtering_coefficient = 0.0f,
        .mode = control::ConstrainedPID::OutputFilter,
    };
}

inline control::ConstrainedPID::PID_Init_t PitchSpeedPid() {
    return {
        .kp = 1200.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .max_out = 12000.0f,
        .max_iout = 2000.0f,
        .deadband = 0.0f,
        .A = 1.5f * PI,
        .B = 1.0f * PI,
        .output_filtering_coefficient = 0.10f,
        .derivative_filtering_coefficient = 0.0f,
        .mode = control::ConstrainedPID::Integral_Limit |
                control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate,
    };
}

inline control::ConstrainedPID::PID_Init_t YawAnglePid() {
    return {
        .kp = 45.0f,
        .ki = 0.0f,
        .kd = 1500.0f,
        .max_out = 6.0f * PI,
        .max_iout = 0.0f,
        .deadband = 0.0f,
        .A = 0.0f,
        .B = 0.0f,
        .output_filtering_coefficient = 0.10f,
        .derivative_filtering_coefficient = 0.0f,
        .mode = control::ConstrainedPID::OutputFilter,
    };
}

inline control::ConstrainedPID::PID_Init_t YawSpeedPid() {
    return {
        .kp = 1200.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .max_out = 12000.0f,
        .max_iout = 2000.0f,
        .deadband = 0.0f,
        .A = 0.5f * PI,
        .B = 0.5f * PI,
        .output_filtering_coefficient = 0.10f,
        .derivative_filtering_coefficient = 0.0f,
        .mode = control::ConstrainedPID::Integral_Limit |
                control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate,
    };
}

}  // namespace xuanshu::gimbal_config
