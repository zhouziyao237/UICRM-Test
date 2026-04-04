#include "shoot_task.h"

#include <cmath>
#include <cstdlib>

#include "MotorCanBase.h"
#include "bsp_os.h"
#include "heat_control.h"
#include "gimbal_task.h"
#include "rc.h"
#include "referee_task.h"
#include "robot_state.h"
#include "sys.h"
#include "utils.h"

osThreadId_t ShootTaskHandle;

namespace {
constexpr uint16_t kLeftFlywheelId = 0x202;
constexpr uint16_t kRightFlywheelId = 0x203;
constexpr uint16_t kFeederId = 0x201;
constexpr float kFlywheelSpeed = 14.0f * PI;
constexpr float kSingleFeedTheta = (2.0f * PI) / 7.0f;
constexpr float kUnloadFeedTheta = PI / 7.0f;
constexpr int16_t kStallSpeedThreshold = 50;
constexpr int16_t kStallCurrentThreshold = 5000;
constexpr uint32_t kStallConfirmMs = 500;

enum FlywheelState_e : uint8_t {
    FLYWHEEL_OFF = 0,
    FLYWHEEL_ON,
};

enum ShootRequestFlag : uint32_t {
    SHOOT_REQUEST_SINGLE = 1U << 0,
    SHOOT_REQUEST_DOUBLE = 1U << 1,
    SHOOT_REQUEST_RAPID = 1U << 2,
    SHOOT_REQUEST_UNLOAD = 1U << 3,
    SHOOT_REQUEST_ALL = SHOOT_REQUEST_SINGLE | SHOOT_REQUEST_DOUBLE |
                        SHOOT_REQUEST_RAPID | SHOOT_REQUEST_UNLOAD,
};

driver::MotorCANBase* flywheel_left = nullptr;
driver::MotorCANBase* flywheel_right = nullptr;
driver::Motor2006* feeder = nullptr;
control::HeatController heat_controller(control::SHOOTER_TYPE_COOL);

FlywheelState_e flywheel_state = FLYWHEEL_OFF;
ShootMode_e shoot_mode = SHOOT_MODE_IDLE;
ShooterTelemetry_t shoot_data{};
uint32_t feeder_stall_start_ms = 0;
BoolEdgeDetector friction_edge(false);
BoolEdgeDetector unload_edge(false);
uint32_t last_rc_generation = 0;
uint32_t last_fire_sequence = 0;

control::ConstrainedPID::PID_Init_t FlywheelPid() {
    return {
        .kp = 100,
        .ki = 1,
        .kd = 0,
        .max_out = 1000,
        .max_iout = 4000,
        .deadband = 0,
        .A = 3 * PI,
        .B = 2 * PI,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::Integral_Limit |
                control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate,
    };
}

control::ConstrainedPID::PID_Init_t FeederAnglePid() {
    return {
        .kp = 30,
        .ki = 0,
        .kd = 0,
        .max_out = 5 * PI,
        .max_iout = 0,
        .deadband = 0,
        .A = 0,
        .B = 0,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::OutputFilter,
    };
}

control::ConstrainedPID::PID_Init_t FeederSpeedPid() {
    return {
        .kp = 200,
        .ki = 0,
        .kd = 10000,
        .max_out = 10000,
        .max_iout = 4000,
        .deadband = 0,
        .A = 3 * PI,
        .B = 2 * PI,
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::Integral_Limit |
                control::ConstrainedPID::OutputFilter |
                control::ConstrainedPID::Trapezoid_Intergral |
                control::ConstrainedPID::ChangingIntegralRate |
                control::ConstrainedPID::ErrorHandle,
    };
}

bool ShooterPowerAllowed() {
#ifdef HAS_REFEREE
    const RefereeSnapshot_t referee = RefereeGetSnapshot();
    if (referee.online && !referee.shooter_power_enabled) return false;
#endif
    return true;
}

bool ShooterMotorsOnline() {
    return feeder != nullptr && feeder->IsOnline() &&
           flywheel_left != nullptr && flywheel_left->IsOnline() &&
           flywheel_right != nullptr && flywheel_right->IsOnline();
}

bool ShooterRunAllowed() {
    const GimbalTelemetry_t gimbal = GimbalGetTelemetry();
    return RobotCanRun() && gimbal.initialized && gimbal.motors_online &&
           ShooterPowerAllowed();
}

void PostShootRequest(uint32_t request) {
    if (ShootTaskHandle != nullptr) {
        osThreadFlagsSet(ShootTaskHandle, request & SHOOT_REQUEST_ALL);
    }
}

void ClearShootRequests() {
    if (ShootTaskHandle != nullptr) {
        osThreadFlagsClear(SHOOT_REQUEST_ALL);
    }
}

ShootMode_e ConsumeShootRequest() {
    const uint32_t flags =
        osThreadFlagsWait(SHOOT_REQUEST_ALL, osFlagsWaitAny, 0);
    if ((flags & osFlagsError) != 0U) return SHOOT_MODE_IDLE;
    if ((flags & SHOOT_REQUEST_UNLOAD) != 0U) return SHOOT_MODE_UNLOAD;
    if ((flags & SHOOT_REQUEST_RAPID) != 0U) return SHOOT_MODE_RAPID;
    if ((flags & SHOOT_REQUEST_DOUBLE) != 0U) return SHOOT_MODE_DOUBLE;
    if ((flags & SHOOT_REQUEST_SINGLE) != 0U) return SHOOT_MODE_SINGLE;
    return SHOOT_MODE_IDLE;
}

void PrimeRemoteInput() {
    const RemoteControl_t remote = RCGetSnapshot();
    friction_edge.reset(remote.friction_toggle);
    unload_edge.reset(remote.unload);
    last_fire_sequence = remote.fire_sequence;
}

void ProcessRemoteInput() {
    const RemoteControl_t remote = RCGetSnapshot();

    friction_edge.input(remote.friction_toggle);
    unload_edge.input(remote.unload);

    if (friction_edge.posEdge()) {
        flywheel_state = flywheel_state == FLYWHEEL_OFF
                             ? FLYWHEEL_ON
                             : FLYWHEEL_OFF;
    }
    if (unload_edge.posEdge()) {
        PostShootRequest(SHOOT_REQUEST_UNLOAD);
    }

    if (remote.fire_sequence != last_fire_sequence) {
        last_fire_sequence = remote.fire_sequence;
        if (flywheel_state == FLYWHEEL_ON && shoot_mode == SHOOT_MODE_IDLE) {
            switch (remote.fire_command) {
                case REMOTE_FIRE_SINGLE:
                    PostShootRequest(SHOOT_REQUEST_SINGLE);
                    break;
                case REMOTE_FIRE_DOUBLE:
                    PostShootRequest(SHOOT_REQUEST_DOUBLE);
                    break;
                case REMOTE_FIRE_RAPID:
                    PostShootRequest(SHOOT_REQUEST_RAPID);
                    break;
                case REMOTE_FIRE_NONE:
                default:
                    break;
            }
        }
    }
}

void ApplyHeatRules() {
#ifdef HAS_REFEREE
    const RefereeSnapshot_t referee = RefereeGetSnapshot();
    if (referee.online) {
        if ((referee.match_running && referee.bullets_17mm == 0) ||
            referee.bullet_speed >= 23.0f) {
            shoot_mode = SHOOT_MODE_IDLE;
        }

        heat_controller.Update(referee.shooter_heat,
                               referee.shooter_heat_limit);
        if (referee.shooter_heat > heat_controller.GetSafeLimit()) {
            shoot_mode = SHOOT_MODE_IDLE;
        }
        return;
    }
#endif
    heat_controller.Update(0, 200);
}

void UpdateFlywheels() {
    if (flywheel_state == FLYWHEEL_OFF) {
        flywheel_left->SetTarget(0.0f);
        flywheel_right->SetTarget(0.0f);
        flywheel_left->Disable();
        flywheel_right->Disable();
        if (shoot_mode != SHOOT_MODE_UNLOAD) shoot_mode = SHOOT_MODE_IDLE;
        return;
    }

    flywheel_left->Enable();
    flywheel_left->SetTarget(-kFlywheelSpeed);
    flywheel_right->Enable();
    flywheel_right->SetTarget(kFlywheelSpeed);
}

bool SafeShootDelay(uint32_t delay_ms) {
    const uint32_t start = bsp::GetHighresTickMilliSec();
    while (bsp::GetHighresTickMilliSec() - start < delay_ms) {
        if (!ShooterRunAllowed()) return false;
        osDelay(5);
    }
    return true;
}

void UnloadFeeder() {
    const float original_target = feeder->GetTarget();
    feeder->SetTarget(original_target + kUnloadFeedTheta);
    if (!SafeShootDelay(200)) return;
    feeder->SetTarget(original_target);
    SafeShootDelay(200);
}

void FeedShots(uint16_t count) {
    for (uint16_t i = 0; i < count; ++i) {
        feeder->SetTarget(feeder->GetTarget() + kSingleFeedTheta);
        heat_controller.RecordShot();
    }
}

void ExecuteShootMode() {
    bool action_finished = false;
    switch (shoot_mode) {
        case SHOOT_MODE_UNLOAD:
            feeder->Enable();
            UnloadFeeder();
            shoot_data.jammed = false;
            action_finished = true;
            break;
        case SHOOT_MODE_SINGLE:
            if (heat_controller.CanShoot()) FeedShots(1);
            action_finished = true;
            break;
        case SHOOT_MODE_DOUBLE:
            if (heat_controller.CanShoot(2)) {
                FeedShots(2);
            } else if (heat_controller.CanShoot()) {
                FeedShots(1);
            }
            action_finished = true;
            break;
        case SHOOT_MODE_RAPID: {
            uint16_t count = heat_controller.GetMaxShootCount();
            if (count > 7) count = 7;
            FeedShots(count);
            action_finished = true;
            break;
        }
        case SHOOT_MODE_IDLE:
        default:
            break;
    }

    if (!action_finished) return;
    shoot_mode = SHOOT_MODE_IDLE;
    if (SafeShootDelay(200)) {
        feeder->Hold();
        feeder->Stop();
    }
}

void DisableShooterOutput() {
    if (flywheel_left != nullptr) {
        flywheel_left->SetTarget(0.0f);
        flywheel_left->Disable();
    }
    if (flywheel_right != nullptr) {
        flywheel_right->SetTarget(0.0f);
        flywheel_right->Disable();
    }
    if (feeder != nullptr) feeder->Disable();

    flywheel_state = FLYWHEEL_OFF;
    shoot_mode = SHOOT_MODE_IDLE;
    shoot_data.output_enabled = false;
    shoot_data.jammed = false;
    feeder_stall_start_ms = 0;
}

void UpdateShooterTelemetry() {
    shoot_data.feeder_online = feeder != nullptr && feeder->IsOnline();
    shoot_data.online = ShooterMotorsOnline();
    shoot_data.friction_enabled = flywheel_state == FLYWHEEL_ON;
    shoot_data.mode = shoot_mode;
}

void FeederUpdateCallback(void* instance) {
    auto* motor = static_cast<driver::Motor2006*>(instance);
    if (motor == nullptr) return;

    const bool stalled =
        std::abs(motor->GetOutput()) > kStallCurrentThreshold &&
        std::abs(motor->GetOmega()) < kStallSpeedThreshold;
    const uint32_t now = bsp::GetHighresTickMilliSec();

    if (!stalled) {
        feeder_stall_start_ms = 0;
        shoot_data.jammed = false;
        return;
    }
    if (feeder_stall_start_ms == 0) {
        feeder_stall_start_ms = now;
        return;
    }
    if (now - feeder_stall_start_ms >= kStallConfirmMs) {
        shoot_data.jammed = true;
        PostShootRequest(SHOOT_REQUEST_UNLOAD);
        feeder_stall_start_ms = now;
    }
}
}  // namespace

void InitShoot() {
    flywheel_left = new driver::Motor3508(atlas_can2, kLeftFlywheelId);
    flywheel_right = new driver::Motor3508(atlas_can2, kRightFlywheelId);

    driver::MotorCANBase* flywheels[] = {flywheel_left, flywheel_right};
    for (auto* motor : flywheels) {
        motor->SetTransmissionRatio(19.0f);
        motor->ReInitPID(FlywheelPid(),
                         driver::MotorCANBase::SPEED_LOOP_CONTROL);
        motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL);
        motor->SetTarget(0.0f);
        motor->Disable();
    }

    feeder = new driver::Motor2006(atlas_can1, kFeederId);
    feeder->SetTransmissionRatio(36.0f);
    feeder->ReInitPID(FeederSpeedPid(),
                      driver::MotorCANBase::SPEED_LOOP_CONTROL);
    feeder->ReInitPID(FeederAnglePid(),
                      driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    feeder->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL |
                    driver::MotorCANBase::SPEED_LOOP_CONTROL);
    feeder->RegisterDataUpdateCallback(FeederUpdateCallback, feeder);
    feeder->Disable();

    PrimeRemoteInput();
    last_rc_generation = RCGetSnapshot().generation;
    UpdateShooterTelemetry();
}

void ShootTask(void* arg) {
    UNUSED(arg);
    RM_ASSERT_TRUE(feeder != nullptr && flywheel_left != nullptr &&
                       flywheel_right != nullptr,
                   "Shooter init failed");

    DisableShooterOutput();
    shoot_data.initialized = true;

    while (true) {
        const RemoteControl_t remote = RCGetSnapshot();
        if (remote.generation != last_rc_generation) {
            last_rc_generation = remote.generation;
            PrimeRemoteInput();
            ClearShootRequests();
        }

        if (!ShooterRunAllowed() || !ShooterMotorsOnline()) {
            DisableShooterOutput();
            ClearShootRequests();
            PrimeRemoteInput();
            UpdateShooterTelemetry();
            osDelay(10);
            continue;
        }

        if (!shoot_data.output_enabled) {
            feeder->Enable();
            shoot_data.output_enabled = true;
        }

        ProcessRemoteInput();
        const ShootMode_e request = ConsumeShootRequest();
        if (request == SHOOT_MODE_UNLOAD ||
            (request != SHOOT_MODE_IDLE && flywheel_state == FLYWHEEL_ON)) {
            shoot_mode = request;
        }

        ApplyHeatRules();
        UpdateFlywheels();
        ExecuteShootMode();
        UpdateShooterTelemetry();
        osDelay(1);
    }
}

uint8_t ShootIsOnline() {
    return ShooterMotorsOnline() ? 1U : 0U;
}

ShooterTelemetry_t ShooterGetTelemetry() {
    return shoot_data;
}
