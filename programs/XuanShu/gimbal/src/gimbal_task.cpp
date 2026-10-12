#include "gimbal_task.h"

#include <cmath>

#include "MotorCanBase.h"
#include "bsp_can.h"
#include "bsp_error_handler.h"
#include "bsp_os.h"
#include "gimbal.h"
#include "gimbal_data.h"
#include "imu_task.h"
#include "main.h"
#include "minipc_task.h"
#include "rc_task.h"
#include "sys_task.h"
#include "cmsis_os.h"
#include "task.h"
#include "utils.h"

namespace cfg = xuanshu::gimbal_config;

namespace {
bsp::CAN* can1 = nullptr;
driver::Motor6020* yaw_motor = nullptr;
driver::Motor6020* pitch_motor = nullptr;
control::Gimbal* gimbal = nullptr;
GimbalTelemetry_t telemetry_frames[2]{};
volatile uint8_t active_telemetry = 0U;
bool output_enabled = false;
GimbalMode previous_mode = GimbalMode::KILL;
bool encoder_hold_synced = false;
uint32_t last_control_ms = 0U;

void PublishTelemetry() {
    GimbalTelemetry_t t;
    t.initialized = gimbal != nullptr;
    t.output_enabled = output_enabled;
    t.pitch_online = pitch_motor != nullptr && pitch_motor->IsOnline();
    t.yaw_online = yaw_motor != nullptr && yaw_motor->IsOnline();

    if (pitch_motor != nullptr) {
        t.pitch_motor_theta = pitch_motor->GetOutputShaftTheta();
        t.pitch_motor_omega = pitch_motor->GetOutputShaftOmega();
        t.pitch_current = pitch_motor->GetCurr();
    }
    if (yaw_motor != nullptr) {
        t.yaw_motor_theta = yaw_motor->GetOutputShaftTheta();
        t.yaw_motor_omega = yaw_motor->GetOutputShaftOmega();
        t.yaw_current = yaw_motor->GetCurr();
    }
    if (gimbal != nullptr) {
        t.pitch_relative = gimbal->getPitchByMotor();
        t.yaw_relative = gimbal->getYawByMotor();
        t.pitch_target = gimbal->getPitchTargetLogical();
        t.yaw_target = gimbal->getYawTarget();
    }

    taskENTER_CRITICAL();
    const uint8_t next = static_cast<uint8_t>(active_telemetry ^ 1U);
    telemetry_frames[next] = t;
    active_telemetry = next;
    taskEXIT_CRITICAL();
}

bool MotorsOnline() {
    return pitch_motor != nullptr && yaw_motor != nullptr &&
           pitch_motor->IsOnline() && yaw_motor->IsOnline();
}

void DisableOutput() {
    // Stop()/PID reset only on the enabled->disabled edge. Repeating Stop()
    // every 2 ms would create unnecessary cross-task PID reset traffic.
    if (pitch_motor != nullptr) {
        if (output_enabled || pitch_motor->IsEnable()) pitch_motor->Stop();
        pitch_motor->Disable();
    }
    if (yaw_motor != nullptr) {
        if (output_enabled || yaw_motor->IsEnable()) yaw_motor->Stop();
        yaw_motor->Disable();
    }
    output_enabled = false;
    encoder_hold_synced = false;
}

void EnableOutput() {
    if (!MotorsOnline()) return;
    if (!pitch_motor->IsEnable()) pitch_motor->Enable();
    if (!yaw_motor->IsEnable()) yaw_motor->Enable();
    output_enabled = true;
}

void SyncEncoderHold() {
    if (gimbal == nullptr) return;
    gimbal->TargetReal(0.0f, 0.0f);
    gimbal->Update();
    if (pitch_motor != nullptr) pitch_motor->Stop();
    if (yaw_motor != nullptr) yaw_motor->Stop();
    encoder_hold_synced = true;
}

bool FinishGimbalConstruction() {
    if (gimbal != nullptr) return true;
    if (!MotorsOnline()) return false;

    control::gimbal_data_t data;
    data.pitch_offset_ = cfg::kUseConfiguredEncoderOffsets
                             ? cfg::kPitchOffsetRad
                             : pitch_motor->GetOutputShaftTheta();
    data.yaw_offset_ = cfg::kUseConfiguredEncoderOffsets
                           ? cfg::kYawOffsetRad
                           : yaw_motor->GetOutputShaftTheta();
    data.pitch_max_ = cfg::kPitchMaxRad;
    data.yaw_max_ = cfg::kYawMaxRad;
    data.yaw_circle_ = cfg::kYawContinuous;
    data.pitch_inverted = cfg::kPitchInverted;
    data.yaw_inverted = cfg::kYawInverted;
    data.pitch_eposition = cfg::kPitchEpositionRad;
    data.yaw_eposition = cfg::kYawEpositionRad;

    control::gimbal_t init = {
        .pitch_motor = pitch_motor,
        .yaw_motor = yaw_motor,
        .data = data,
    };
    gimbal = new control::Gimbal(init);
    RM_ASSERT_TRUE(gimbal != nullptr && gimbal->GetData() != nullptr,
                   "XuanShu Gimbal construct failed");
    SyncEncoderHold();
    DisableOutput();
    return true;
}

void ConfigureMotors() {
    pitch_motor->SetTransmissionRatio(cfg::kMotorTransmissionRatio);
    yaw_motor->SetTransmissionRatio(cfg::kMotorTransmissionRatio);

    pitch_motor->ReInitPID(cfg::PitchAnglePid(),
                           driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    pitch_motor->ReInitPID(cfg::PitchSpeedPid(),
                           driver::MotorCANBase::SPEED_LOOP_CONTROL);
    pitch_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL |
                         driver::MotorCANBase::SPEED_LOOP_CONTROL |
                         driver::MotorCANBase::SINGLETURN);

    yaw_motor->ReInitPID(cfg::YawAnglePid(),
                         driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    yaw_motor->ReInitPID(cfg::YawSpeedPid(),
                         driver::MotorCANBase::SPEED_LOOP_CONTROL);
    yaw_motor->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL |
                       driver::MotorCANBase::SPEED_LOOP_CONTROL |
                       (cfg::kYawContinuous ? driver::MotorCANBase::ABSOLUTE
                                            : driver::MotorCANBase::SINGLETURN));

    pitch_motor->Disable();
    yaw_motor->Disable();
}

float ControlDtSeconds() {
    const uint32_t now = bsp::GetHighresTickMilliSec();
    float dt = static_cast<float>(cfg::kGimbalPeriodMs) * 0.001f;
    if (last_control_ms != 0U && now > last_control_ms) {
        const uint32_t elapsed = now - last_control_ms;
        if (elapsed <= 20U) dt = static_cast<float>(elapsed) * 0.001f;
    }
    last_control_ms = now;
    return dt;
}

void RunManual(const RcSnapshot_t& rc, float dt) {
    if (!encoder_hold_synced) SyncEncoderHold();
    gimbal->TargetRel(rc.pitch_rate_rad_s * dt, rc.yaw_rate_rad_s * dt);
    gimbal->Update();
}

void RunAutoAim(const ImuData_t& imu, const VisionTargetSnapshot_t& vision) {
    if (!imu.online || !imu.calibrated || !vision.valid) {
        if (!encoder_hold_synced) SyncEncoderHold();
        gimbal->Update();
        return;
    }

    float pitch_target = vision.pitch;
    float yaw_target = vision.yaw;
    if (!cfg::kVisionTargetIsAbsoluteWorld) {
        pitch_target += imu.pitch;
        yaw_target += imu.yaw;
    }

    encoder_hold_synced = false;
    gimbal->TargetAbs(pitch_target, yaw_target);
    gimbal->UpdateIMU(imu.pitch, imu.yaw);
}
}  // namespace

osThreadId_t GimbalTaskHandle;

void InitGimbal() {
    can1 = new bsp::CAN(&hcan1, true);
    RM_ASSERT_TRUE(can1 != nullptr, "XuanShu CAN1 init failed");

    yaw_motor = new driver::Motor6020(can1, cfg::kYawMotorRxId);
    pitch_motor = new driver::Motor6020(can1, cfg::kPitchMotorRxId);
    RM_ASSERT_TRUE(yaw_motor != nullptr && pitch_motor != nullptr,
                   "XuanShu M6020 init failed");
    ConfigureMotors();
}

void GimbalTask(void* arg) {
    UNUSED(arg);
    DisableOutput();

    const uint32_t start = bsp::GetHighresTickMilliSec();
    while (!FinishGimbalConstruction() &&
           bsp::GetHighresTickMilliSec() - start < cfg::kMotorStartupTimeoutMs) {
        PublishTelemetry();
        osDelay(5U);
    }

    while (true) {
        const AppStateSnapshot_t state = AppStateGetSnapshot();
        const RcSnapshot_t rc = RCGetSnapshot();
        const ImuData_t imu = ImuGetData();
        const VisionTargetSnapshot_t vision = VisionGetSnapshot();
        const float dt = ControlDtSeconds();

        if (!FinishGimbalConstruction() || !MotorsOnline() ||
            state.mode == GimbalMode::KILL) {
            DisableOutput();
            previous_mode = GimbalMode::KILL;
            PublishTelemetry();
            osDelay(cfg::kGimbalPeriodMs);
            continue;
        }

        EnableOutput();
        if (state.mode != previous_mode) {
            SyncEncoderHold();
            previous_mode = state.mode;
        }

        switch (state.mode) {
            case GimbalMode::MANUAL:
                RunManual(rc, dt);
                break;
            case GimbalMode::AUTOAIM:
                RunAutoAim(imu, vision);
                break;
            case GimbalMode::KILL:
            default:
                DisableOutput();
                break;
        }

        PublishTelemetry();
        osDelay(cfg::kGimbalPeriodMs);
    }
}

GimbalTelemetry_t GimbalGetTelemetry() {
    taskENTER_CRITICAL();
    const GimbalTelemetry_t t = telemetry_frames[active_telemetry];
    taskEXIT_CRITICAL();
    return t;
}
