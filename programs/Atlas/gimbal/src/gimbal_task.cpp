#include "gimbal_task.h"

#include <cmath>

#include "MotorCanBase.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "gimbal.h"
#include "imu_task.h"
#include "rc.h"
#include "referee_task.h"
#include "robot_state.h"
#include "sys.h"
#include "utils.h"

osThreadId_t GimbalTaskHandle;

namespace {
constexpr float kPitchTransmissionRatio = 6.0f;
// Measured on the real Atlas: the muzzle is level at this continuous
// M3508 output-shaft coordinate.
constexpr float kPitchLevelCoordinate = 0.5517f;
// The two-stage transmission makes motor-positive and gimbal-positive
// directions opposite. Keep this sign conversion in one configuration flag.
constexpr bool kPitchInverted = true;
constexpr float kPitchTotalTravelRad = 45.0f * PI / 180.0f;
constexpr float kPitchHalfTravelRad = kPitchTotalTravelRad * 0.5f;
constexpr uint32_t kHomeStableTimeMs = 500;
constexpr uint32_t kCenteringTimeMs = 1800;
constexpr uint32_t kCalibrationTimeoutMs = 10000;
constexpr float kHomeMaxOmega = 0.03f;

// Module-owned objects. Other tasks only read GimbalTelemetry_t.
driver::Motor3508* pitch_motor = nullptr;
driver::Motor6020* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::gimbal_data_t* gimbal_params = nullptr;

GimbalTelemetry_t gimbal_data{};
bool gimbal_output_enabled = false;
RemoteMode_e previous_mode = REMOTE_MODE_KILL;
float world_pitch_target = 0.0f;
float world_yaw_target = 0.0f;
uint32_t pitch_stall_start_ms = 0;
uint32_t pitch_last_report_ms = 0;

control::ConstrainedPID::PID_Init_t PitchAnglePid() {
    return {
        .kp = 18,
        .ki = 0,
        .kd = 2,
        .max_out = 16 * PI,
        .max_iout = 0,
        .deadband = 0,
        .A = 0,
        .B = 0,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::OutputFilter,
    };
}

control::ConstrainedPID::PID_Init_t PitchSpeedPid() {
    return {
        .kp = 3000,
        .ki = 0,
        .kd = 0,
        .max_out = 12000,
        .max_iout = 6000,
        .deadband = 0,
        .A = 1.5 * PI,
        .B = 1 * PI,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::Integral_Limit |
                control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate,
    };
}

control::ConstrainedPID::PID_Init_t YawAnglePid() {
    return {
        .kp = 60,
        .ki = 0,
        .kd = 4000,
        .max_out = 6 * PI,
        .max_iout = 0,
        .deadband = 0,
        .A = 0,
        .B = 0,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::OutputFilter,
    };
}

control::ConstrainedPID::PID_Init_t YawSpeedPid() {
    return {
        .kp = 1500,
        .ki = 0,
        .kd = 0,
        .max_out = 16384,
        .max_iout = 2000,
        .deadband = 0,
        .A = 0.5 * PI,
        .B = 0.5 * PI,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::Integral_Limit |
                control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate,
    };
}

bool GimbalMotorsOnline() {
    return pitch_motor != nullptr && yaw_motor != nullptr &&
           pitch_motor->IsOnline() && yaw_motor->IsOnline();
}

bool GimbalPowerAllowed() {
#ifdef HAS_REFEREE
    const RefereeSnapshot_t referee = RefereeGetSnapshot();
    if (referee.online && !referee.gimbal_power_enabled) return false;
#endif
    return true;
}

bool GimbalRunAllowed() {
    return RobotCanRun() && GimbalPowerAllowed() && GimbalMotorsOnline() &&
           ImuIsOnline();
}

void UpdateGimbalOnline() {
    gimbal_data.pitch_online = pitch_motor != nullptr && pitch_motor->IsOnline();
    gimbal_data.yaw_online = yaw_motor != nullptr && yaw_motor->IsOnline();
    gimbal_data.motors_online = gimbal_data.pitch_online && gimbal_data.yaw_online;
}

void DisableGimbalOutput() {
    // Keep the software target attached to the current mechanism position
    // while KILL/power-off is active. This removes stale targets and prevents
    // a jump when MANUAL is enabled again.
    if (gimbal != nullptr && pitch_motor != nullptr && yaw_motor != nullptr) {
        gimbal->TargetReal(0.0f, 0.0f);
        gimbal->Update();
    }
    if (pitch_motor != nullptr) {
        pitch_motor->Stop();
        pitch_motor->Disable();
    }
    if (yaw_motor != nullptr) {
        yaw_motor->Stop();
        yaw_motor->Disable();
    }
    gimbal_output_enabled = false;
    gimbal_data.output_enabled = false;
}

void EnableGimbalOutput() {
    pitch_motor->Enable();
    yaw_motor->Enable();
    gimbal_output_enabled = true;
    gimbal_data.output_enabled = true;
}

float LimitPitch(float value) {
    return clip<float>(value, -gimbal_params->pitch_max_,
                       gimbal_params->pitch_max_);
}

float LimitYaw(float value) {
    return gimbal_params->yaw_circle_
               ? wrap<float>(value, -PI, PI)
               : clip<float>(value, -gimbal_params->yaw_max_,
                             gimbal_params->yaw_max_);
}

void SyncWorldTarget() {
    const ImuData_t imu = ImuGetData();
    const float pitch = imu.pitch;
    float yaw = imu.yaw;
    if (gimbal_params->yaw_inverted) yaw = -yaw;
    world_pitch_target = LimitPitch(pitch);
    world_yaw_target = LimitYaw(yaw);
}

void SyncManualTarget() {
    if (gimbal == nullptr || pitch_motor == nullptr || yaw_motor == nullptr) {
        return;
    }

    // Attach both motor targets to the current mechanism pose and clear the
    // old mode's PID history. Without this, FOLLOW/SPIN leaves an absolute
    // world target active when MANUAL begins, producing several amperes of
    // stale correction and a reaction torque on the chassis.
    gimbal->TargetReal(0.0f, 0.0f);
    gimbal->Update();
    pitch_motor->Stop();
    yaw_motor->Stop();
}

void UpdateWorldControl() {
    const ImuData_t imu = ImuGetData();
    gimbal->TargetAbs(world_pitch_target, world_yaw_target);
    gimbal->UpdateIMU(imu.pitch, imu.yaw);
}

bool WaitForPitchReset(float coordinate) {
    pitch_motor->ResetOutputShaftTheta(coordinate);
    const uint32_t start = bsp::GetHighresTickMilliSec();
    while (pitch_motor->IsOutputShaftThetaResetPending()) {
        if (!GimbalMotorsOnline() ||
            bsp::GetHighresTickMilliSec() - start > 100) {
            return false;
        }
        osDelay(1);
    }
    return true;
}

void UpdateGimbalTelemetry() {
    UpdateGimbalOnline();
    gimbal_data.output_enabled = gimbal_output_enabled;
    if (gimbal == nullptr || pitch_motor == nullptr || yaw_motor == nullptr) {
        return;
    }
    gimbal_data.pitch_relative = gimbal->getPitchByMotor();
    // control::Gimbal exposes the raw yaw motor coordinate. Atlas has an
    // inverted yaw motor, so convert it once here before chassis FOLLOW/field
    // transforms consume it. Telemetry and all application code now use the
    // logical gimbal direction.
    const float yaw_motor_relative = gimbal->getYawByMotor();
    const float logical_yaw =
        gimbal_params->yaw_inverted ? -yaw_motor_relative : yaw_motor_relative;
    // Application telemetry is always one turn even after hours of SPIN.
    gimbal_data.yaw_relative = wrap<float>(logical_yaw, -PI, PI);
    gimbal_data.pitch_target = gimbal->getPitchTargetLogical();
    gimbal_data.yaw_target = gimbal->getYawTarget();
    gimbal_data.pitch_motor_theta = pitch_motor->GetOutputShaftTheta();
    gimbal_data.yaw_motor_theta = yaw_motor->GetOutputShaftTheta();
    gimbal_data.pitch_motor_omega = pitch_motor->GetOutputShaftOmega();
    gimbal_data.pitch_current = pitch_motor->GetCurr();
}

bool HomeAndCalibrate() {
    uint32_t stable_since = 0;
    uint32_t last_warning = 0;

    for (;;) {
        UpdateGimbalOnline();
        if (!GimbalRunAllowed()) {
            DisableGimbalOutput();
            stable_since = 0;
            osDelay(10);
            continue;
        }

        const float omega = pitch_motor->GetOutputShaftOmega();
        const uint32_t now = bsp::GetHighresTickMilliSec();
        if (fabsf(omega) > kHomeMaxOmega) {
            stable_since = 0;
            if (now - last_warning >= 500) {
                print("Pitch home: place at lower stop and hold still; omega=%.4f\r\n",
                      omega);
                last_warning = now;
            }
            osDelay(10);
            continue;
        }

        if (stable_since == 0) stable_since = now;
        if (now - stable_since < kHomeStableTimeMs) {
            osDelay(10);
            continue;
        }

        // The lower mechanical stop is always logical -pitch_max. Convert
        // that logical direction once into the motor/output-shaft coordinate.
        const float lower_command = -gimbal_params->pitch_max_;
        const float lower_motor_delta = gimbal_params->pitch_inverted
                                            ? -lower_command
                                            : lower_command;
        const float lower_coordinate =
            gimbal_params->pitch_offset_ + lower_motor_delta;
        if (!WaitForPitchReset(lower_coordinate)) {
            print("Pitch re-zero timeout\r\n");
            DisableGimbalOutput();
            stable_since = 0;
            continue;
        }

        gimbal->TargetAbs(lower_command, 0.0f);
        gimbal->Update();
        EnableGimbalOutput();

        const uint32_t center_start = bsp::GetHighresTickMilliSec();
        bool aborted = false;
        while (bsp::GetHighresTickMilliSec() - center_start < kCenteringTimeMs) {
            if (!GimbalRunAllowed()) {
                aborted = true;
                break;
            }
            const float progress = clip<float>(
                static_cast<float>(bsp::GetHighresTickMilliSec() - center_start) /
                    static_cast<float>(kCenteringTimeMs),
                0.0f, 1.0f);
            const float smooth = progress * progress * (3.0f - 2.0f * progress);
            gimbal->TargetAbs(lower_command * (1.0f - smooth), 0.0f);
            gimbal->Update();
            osDelay(1);
        }
        if (aborted) {
            DisableGimbalOutput();
            stable_since = 0;
            continue;
        }

        BuzzerPlay(SingCaliStart);
        ImuStartCalibration();
        const uint32_t calibration_start = bsp::GetHighresTickMilliSec();
        while (!ImuCalibrationDone()) {
            if (!GimbalRunAllowed() ||
                bsp::GetHighresTickMilliSec() - calibration_start >
                    kCalibrationTimeoutMs) {
                aborted = true;
                break;
            }
            gimbal->TargetAbs(0.0f, 0.0f);
            gimbal->Update();
            osDelay(1);
        }
        if (aborted) {
            print("IMU calibration interrupted or timed out\r\n");
            BuzzerPlay(SingError);
            DisableGimbalOutput();
            stable_since = 0;
            continue;
        }

        BuzzerPlay(SingCaliDone);
        UpdateGimbalTelemetry();
        return true;
    }
}

void MonitorPitchStall() {
    constexpr int16_t kHighCurrent = 10000;
    constexpr float kLowSpeed = 0.05f;
    constexpr float kMinError = 0.08f;
    constexpr uint32_t kConfirmMs = 300;

    const auto state =
        pitch_motor->GetPIDState(driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    const bool suspected =
        std::abs(pitch_motor->GetCurr()) >= kHighCurrent &&
        fabsf(pitch_motor->GetOutputShaftOmega()) <= kLowSpeed &&
        fabsf(state.error) >= kMinError;
    const uint32_t now = bsp::GetHighresTickMilliSec();

    if (!suspected) {
        pitch_stall_start_ms = 0;
        return;
    }
    if (pitch_stall_start_ms == 0) pitch_stall_start_ms = now;
    if (now - pitch_stall_start_ms >= kConfirmMs &&
        now - pitch_last_report_ms >= 1000) {
        print("Pitch stall: theta=%.4f target=%.4f omega=%.4f current=%d error=%.4f\r\n",
              pitch_motor->GetOutputShaftTheta(), pitch_motor->GetTarget(),
              pitch_motor->GetOutputShaftOmega(), pitch_motor->GetCurr(),
              state.error);
        pitch_last_report_ms = now;
    }
}

void RunGimbalMode(RemoteMode_e mode, const RemoteControl_t& remote) {
    // Do not feed the commanded chassis vt into the yaw motor. The gimbal
    // board has no measured chassis angular velocity; when the lower chassis
    // is disabled, blocked or misses a command, command-based feed-forward
    // rotates the head even though the chassis did not move. IMU world-angle
    // feedback is authoritative here.
    yaw_motor->SetSpeedOffset(0.0f);

    switch (mode) {
        case REMOTE_MODE_MANUAL:
            gimbal->TargetRel(remote.gimbal_pitch_delta,
                              remote.gimbal_yaw_delta);
            gimbal->Update();
            break;

        case REMOTE_MODE_FOLLOW:
        case REMOTE_MODE_SPIN:
            world_pitch_target =
                LimitPitch(world_pitch_target + remote.gimbal_pitch_delta);
            world_yaw_target =
                LimitYaw(world_yaw_target + remote.gimbal_yaw_delta);
            UpdateWorldControl();
            break;

        case REMOTE_MODE_AUTOAIM:
            UpdateWorldControl();
            break;

        case REMOTE_MODE_AUTO:
            // Add autonomous target logic here.
            gimbal->Update();
            break;

        case REMOTE_MODE_KILL:
        default:
            DisableGimbalOutput();
            break;
    }
}
}  // namespace

void InitGimbal() {
    pitch_motor = new driver::Motor3508(atlas_can2, 0x204);
    yaw_motor = new driver::Motor6020(atlas_can1, 0x205);

    pitch_motor->SetTransmissionRatio(kPitchTransmissionRatio);
    pitch_motor->ReInitPID(PitchAnglePid(),
                           driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    pitch_motor->ReInitPID(PitchSpeedPid(),
                           driver::MotorCANBase::SPEED_LOOP_CONTROL);
    pitch_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                         driver::MotorCANBase::ANGLE_LOOP_CONTROL);

    yaw_motor->SetTransmissionRatio(1.0f);
    yaw_motor->ReInitPID(YawAnglePid(),
                         driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    yaw_motor->ReInitPID(YawSpeedPid(),
                         driver::MotorCANBase::SPEED_LOOP_CONTROL);
    yaw_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                       driver::MotorCANBase::ANGLE_LOOP_CONTROL |
                       driver::MotorCANBase::ABSOLUTE);

    control::gimbal_t config;
    config.pitch_motor = pitch_motor;
    config.yaw_motor = yaw_motor;
    config.data = {
        .pitch_offset_ = kPitchLevelCoordinate,
        .yaw_offset_ = -1.9853f,
        .pitch_max_ = kPitchHalfTravelRad,
        .yaw_max_ = PI,
        .yaw_circle_ = true,
        .pitch_inverted = kPitchInverted,
        .yaw_inverted = true,
    };
    gimbal = new control::Gimbal(config);
    gimbal_params = gimbal->GetData();

    RM_ASSERT_TRUE(pitch_motor != nullptr && yaw_motor != nullptr &&
                       gimbal != nullptr && gimbal_params != nullptr,
                   "Gimbal init failed");
    DisableGimbalOutput();
}

void GimbalTask(void* arg) {
    UNUSED(arg);

    if (!HomeAndCalibrate()) {
        RM_ASSERT_TRUE(false, "Gimbal calibration failed");
    }

    gimbal_data.initialized = true;
    SyncWorldTarget();
    previous_mode = remote_mode;

    while (true) {
        UpdateGimbalOnline();

        if (!GimbalRunAllowed()) {
            DisableGimbalOutput();
            UpdateGimbalTelemetry();
            osDelay(10);
            continue;
        }

        if (!gimbal_output_enabled) {
            EnableGimbalOutput();
            if (remote_mode == REMOTE_MODE_FOLLOW ||
                remote_mode == REMOTE_MODE_SPIN ||
                remote_mode == REMOTE_MODE_AUTOAIM) {
                SyncWorldTarget();
            } else {
                SyncManualTarget();
            }
            previous_mode = remote_mode;
        }

        if (remote_mode != previous_mode) {
            if (remote_mode == REMOTE_MODE_FOLLOW ||
                remote_mode == REMOTE_MODE_SPIN ||
                remote_mode == REMOTE_MODE_AUTOAIM) {
                SyncWorldTarget();
                pitch_motor->Stop();
                yaw_motor->Stop();
            } else if (remote_mode == REMOTE_MODE_MANUAL) {
                SyncManualTarget();
            }
            previous_mode = remote_mode;
        }

        RunGimbalMode(remote_mode, RCGetSnapshot());
        UpdateGimbalTelemetry();
        MonitorPitchStall();
        osDelay(10);
    }
}

uint8_t GimbalIsOnline() {
    return GimbalMotorsOnline() ? 1U : 0U;
}

GimbalTelemetry_t GimbalGetTelemetry() {
    return gimbal_data;
}

float GimbalGetRelativeYaw() {
    return gimbal_data.yaw_relative;
}
