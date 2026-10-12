#include "chassis_task.h"

#include <cmath>

#include "MotorCanBase.h"
#include "atlas_config.h"
#include "bsp_can.h"
#include "can_bridge.h"
#include "chassis.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "main.h"
#include "rc.h"
#include "referee_task.h"
#include "robot_state.h"
#include "sys.h"
#include "utils.h"

osThreadId_t ChassisTaskHandle;

namespace {
// The chassis board already limits all wheel targets to 2 * 7 * PI. Keep the
// sender in the same range instead of sending 4x oversized values and relying
// on a hidden second normalization step on the lower board.
constexpr float kMaxXySpeed = 2.0f * PI * 7.0f;
constexpr float kMaxTurnSpeed = 2.0f * PI * 7.0f;

// Runtime copy of the installation calibration.  Keep the public setting in
// degrees so +90/-90 tests are easy to understand.  A 32-bit aligned float is
// read and written atomically on Cortex-M4; writes are accepted only while the
// chassis output is disabled.
volatile float g_chassis_frame_offset_deg = kDefaultChassisFrameOffsetDeg;

// FOLLOW uses a deliberately small direct P controller. Positive chassis turn
// makes the measured Atlas gimbal-relative yaw grow, so the correction sign is
// negative. This sign is confirmed by the real-car serial trace.
constexpr float kFollowYawDeadband = 0.035f;
constexpr float kFollowTurnKp = 0.32f;          // normalized command / rad
constexpr float kFollowMaxTurnCommand = 0.22f;  // normalized command

// SPIN is a persistent one-direction small-gyro mode. The left dial only
// changes direction when it is moved far enough; center noise cannot make the
// robot alternate left/right. Reserve wheel-speed headroom for rotation so the
// lower chassis does not repeatedly L1-normalize the rotating x/y vector into
// a four-lobed (square-like) motion.
constexpr float kSpinTurnCommand = 0.35f;
constexpr float kSpinMaxTranslationCommand = 0.45f;
constexpr float kSpinDirectionThreshold = 0.18f;

// Manual straight-line heading hold compensates wheel/motor asymmetry. The
// heading estimate removes gimbal relative yaw from the gimbal-mounted IMU yaw.
constexpr float kHeadingHoldMinTranslation = 0.12f;
constexpr float kHeadingHoldTurnDeadband = 0.10f;
constexpr float kHeadingHoldKp = 0.30f;          // normalized command / rad
constexpr float kHeadingHoldMaxCommand = 0.10f;  // normalized command

communication::CanBridge* g_can_bridge = nullptr;
control::ChassisCanBridgeSender* g_chassis = nullptr;
control::ChassisCanBridgeSender* g_power_sender = nullptr;

// Translation is rate-limited as one 2-D vector.  Independent X/Y Ease
// objects distort a rotating command into axis-aligned/diamond transients.
constexpr float kTranslationVectorStep = 0.9f;
float g_smoothed_vx = 0.0f;
float g_smoothed_vy = 0.0f;
Ease g_ease_vt(0.0f, 0.9f);

ChassisTelemetry_t g_telemetry{};
RemoteMode_e g_previous_mode = REMOTE_MODE_KILL;

bool g_relative_yaw_initialized = false;
float g_previous_relative_yaw = 0.0f;
float g_relative_yaw_continuous = 0.0f;

float g_spin_direction = 1.0f;
bool g_heading_hold_active = false;
float g_heading_target = 0.0f;

float WrapPi(float angle) {
    return wrap<float>(angle, -PI, PI);
}

float SignOr(float value, float fallback) {
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return fallback;
}

/**
 * Convert Atlas logical translation into the chassis-library convention.
 *
 * Atlas logical axes:
 *   forward > 0 : move toward the configured robot front
 *   right   > 0 : move toward the configured robot right
 *
 * control::Chassis::SetSpeed axes:
 *   x_speed > 0 : right
 *   y_speed > 0 : forward
 *
 * The configurable frame offset is applied in forward/right coordinates, then
 * the result is reordered to x=right, y=forward for the lower chassis library.
 */
void MapLogicalToChassisLibrary(float forward, float right,
                                float& library_x_speed,
                                float& library_y_speed) {
    const float offset_rad =
        static_cast<float>(g_chassis_frame_offset_deg) * PI / 180.0f;
    const float sin_offset = arm_sin_f32(offset_rad);
    const float cos_offset = arm_cos_f32(offset_rad);

    const float calibrated_forward =
        cos_offset * forward + sin_offset * right;
    const float calibrated_right =
        -sin_offset * forward + cos_offset * right;

    library_x_speed = calibrated_right;
    library_y_speed = calibrated_forward;
}

bool ChassisPowerAllowed() {
#ifdef HAS_REFEREE
    const RefereeSnapshot_t referee = RefereeGetSnapshot();
    if (referee.online && !referee.chassis_power_enabled) return false;
#endif
    return true;
}

void ResetYawTracking() {
    g_relative_yaw_initialized = false;
    g_previous_relative_yaw = 0.0f;
    g_relative_yaw_continuous = 0.0f;
    g_heading_hold_active = false;
    g_heading_target = 0.0f;
    g_telemetry.relative_yaw_continuous = 0.0f;
    g_telemetry.heading_hold_active = false;
    g_telemetry.heading_error = 0.0f;
    g_telemetry.follow_error = 0.0f;
}

void ResetMotion() {
    g_telemetry.vx = 0.0f;
    g_telemetry.vy = 0.0f;
    g_telemetry.vt = 0.0f;
    g_telemetry.operator_x = 0.0f;
    g_telemetry.operator_y = 0.0f;
    g_telemetry.operator_turn = 0.0f;
    g_telemetry.transformed_x = 0.0f;
    g_telemetry.transformed_y = 0.0f;
    g_telemetry.library_x_speed = 0.0f;
    g_telemetry.library_y_speed = 0.0f;
    g_telemetry.frame_offset_deg =
        static_cast<float>(g_chassis_frame_offset_deg);
    g_smoothed_vx = 0.0f;
    g_smoothed_vy = 0.0f;
    g_ease_vt.Reset(0.0f);
    ResetYawTracking();
    if (g_chassis != nullptr) {
        g_chassis->SetSpeed(0.0f, 0.0f, 0.0f);
    }
}

void DisableOutput() {
    if (g_chassis != nullptr) g_chassis->Disable();
    ResetMotion();
    g_telemetry.output_enabled = false;
    g_telemetry.online = false;
}

void ReadOperatorCommand(float& x, float& y, float& turn) {
    const RemoteControl_t remote = RCGetSnapshot();
    if (!remote.online) {
        x = 0.0f;
        y = 0.0f;
        turn = 0.0f;
        return;
    }
    x = remote.chassis_forward;
    y = remote.chassis_lateral;
    turn = remote.chassis_turn;
}

float UpdateContinuousRelativeYaw(float wrapped_yaw) {
    if (!g_relative_yaw_initialized) {
        g_relative_yaw_initialized = true;
        g_previous_relative_yaw = wrapped_yaw;
        g_relative_yaw_continuous = wrapped_yaw;
        return g_relative_yaw_continuous;
    }

    const float adjacent_delta = WrapPi(wrapped_yaw - g_previous_relative_yaw);
    g_relative_yaw_continuous += adjacent_delta;
    g_previous_relative_yaw = wrapped_yaw;
    return g_relative_yaw_continuous;
}

bool GetChassisHeading(float relative_yaw, float& heading) {
    const ImuData_t imu = ImuGetData();
    if (!imu.online || !imu.calibrated || !GimbalIsOnline()) {
        return false;
    }

    // Atlas IMU yaw is already in the physical gimbal yaw direction. The
    // relative-yaw telemetry is now exposed in logical gimbal direction.
    // chassis_world_yaw = gimbal_world_yaw - gimbal_relative_yaw.
    heading = WrapPi(imu.yaw - relative_yaw);
    return true;
}

void HandleModeTransition(RemoteMode_e mode, float command_turn,
                          float relative_yaw) {
    if (mode == g_previous_mode) return;

    // Do not carry the old mode's turn ramp into the new mode.
    g_ease_vt.Reset(0.0f);
    g_heading_hold_active = false;
    g_telemetry.heading_hold_active = false;
    g_telemetry.heading_error = 0.0f;
    g_telemetry.follow_error = 0.0f;

    // Rebase the continuous relative-yaw tracker on every mode transition.
    g_relative_yaw_initialized = true;
    g_previous_relative_yaw = relative_yaw;
    g_relative_yaw_continuous = relative_yaw;

    if (mode == REMOTE_MODE_SPIN &&
        fabsf(command_turn) >= kSpinDirectionThreshold) {
        g_spin_direction = SignOr(command_turn, g_spin_direction);
    }

    g_previous_mode = mode;
}

float ManualTurnCommand(float command_x, float command_y, float command_turn,
                        float relative_yaw) {
    float current_heading = 0.0f;
    const bool heading_valid = GetChassisHeading(relative_yaw, current_heading);
    g_telemetry.chassis_heading = current_heading;

    const float translation = sqrtf(command_x * command_x + command_y * command_y);
    const bool operator_turning = fabsf(command_turn) >= kHeadingHoldTurnDeadband;

    if (!heading_valid || translation < kHeadingHoldMinTranslation ||
        operator_turning) {
        g_heading_hold_active = false;
        g_telemetry.heading_hold_active = false;
        g_telemetry.heading_error = 0.0f;
        if (heading_valid) {
            g_heading_target = current_heading;
            g_telemetry.heading_target = g_heading_target;
        }
        return command_turn;
    }

    if (!g_heading_hold_active) {
        g_heading_target = current_heading;
        g_heading_hold_active = true;
    }

    const float error = WrapPi(g_heading_target - current_heading);
    const float correction = clip<float>(
        -kHeadingHoldKp * error, -kHeadingHoldMaxCommand,
        kHeadingHoldMaxCommand);

    g_telemetry.heading_target = g_heading_target;
    g_telemetry.heading_error = error;
    g_telemetry.heading_hold_active = true;
    return correction;
}

float FollowTurnCommand(float continuous_relative_yaw) {
    float error = continuous_relative_yaw;
    if (fabsf(error) < kFollowYawDeadband) error = 0.0f;

    // Real-car trace confirms positive chassis turn increases Atlas relative
    // yaw. Therefore the centering correction must use the negative sign.
    const float command = clip<float>(
        -kFollowTurnKp * error, -kFollowMaxTurnCommand,
        kFollowMaxTurnCommand);
    g_telemetry.follow_error = error;
    return command;
}

void LimitVectorMagnitude(float& x, float& y, float max_magnitude) {
    const float magnitude = sqrtf(x * x + y * y);
    if (magnitude <= max_magnitude || magnitude <= 0.0f) return;
    const float scale = max_magnitude / magnitude;
    x *= scale;
    y *= scale;
}

float SpinTurnCommand(float operator_turn) {
    if (fabsf(operator_turn) >= kSpinDirectionThreshold) {
        g_spin_direction = SignOr(operator_turn, g_spin_direction);
    }
    return g_spin_direction * kSpinTurnCommand;
}
}  // namespace

bool ChassisSetFrameOffsetDeg(float offset_deg) {
    if (!std::isfinite(offset_deg) || g_telemetry.output_enabled) {
        return false;
    }

    g_chassis_frame_offset_deg =
        wrap<float>(offset_deg, -180.0f, 180.0f);
    g_telemetry.frame_offset_deg =
        static_cast<float>(g_chassis_frame_offset_deg);
    return true;
}

float ChassisGetFrameOffsetDeg() {
    return static_cast<float>(g_chassis_frame_offset_deg);
}

void InitChassis() {
    g_can_bridge = new communication::CanBridge(atlas_can1, 0x51);
    g_chassis = new control::ChassisCanBridgeSender(g_can_bridge, 0x52);
    g_chassis->SetChassisRegId(0x70, 0x71, 0x72, 0x73);
    g_chassis->Disable();

    // Power forwarding owns an independent sender object.  ChassisTask and
    // PowerTask may therefore publish concurrently without sharing the
    // sender's mutable frame scratch buffer.  Both still use the R1 CAN
    // latest-value scheduler underneath.
    g_power_sender = new control::ChassisCanBridgeSender(g_can_bridge, 0x52);
    g_power_sender->SetChassisRegId(0x70, 0x71, 0x72, 0x73);

    g_telemetry.max_xy_speed = kMaxXySpeed;
    g_telemetry.max_turn_speed = kMaxTurnSpeed;
    g_telemetry.frame_offset_deg = ChassisGetFrameOffsetDeg();
}

void ChassisTask(void* arg) {
    UNUSED(arg);
    RM_ASSERT_TRUE(g_chassis != nullptr, "Chassis init failed");

    DisableOutput();
    g_telemetry.initialized = true;
    osDelay(1000);

    while (true) {
        // Freeze one robot-state generation for this whole control iteration.
        // SysTask may publish a newer mode meanwhile, but this cycle never mixes
        // transition logic from one mode with output logic from another.
        const RobotStateSnapshot_t robot_state = RobotGetSnapshot();
        const RemoteMode_e mode = robot_state.mode;
        const bool robot_can_run =
            mode != REMOTE_MODE_KILL && !robot_state.hp_dead;

        if (!robot_can_run || !ChassisPowerAllowed()) {
            DisableOutput();
            g_previous_mode = REMOTE_MODE_KILL;
            osDelay(5);
            continue;
        }

        if (!g_telemetry.output_enabled) {
            g_chassis->Enable();
            ResetMotion();
            g_telemetry.output_enabled = true;
            g_previous_mode = mode;
        }

        float command_x = 0.0f;
        float command_y = 0.0f;
        float command_turn = 0.0f;
        ReadOperatorCommand(command_x, command_y, command_turn);

        // GimbalGetRelativeYaw() is a logical angle: positive has the same
        // meaning as positive operator yaw, independent of yaw motor mounting.
        const float relative_yaw = WrapPi(GimbalGetRelativeYaw());
        HandleModeTransition(mode, command_turn, relative_yaw);
        const float continuous_relative_yaw =
            UpdateContinuousRelativeYaw(relative_yaw);
        g_telemetry.relative_yaw = relative_yaw;
        g_telemetry.relative_yaw_continuous = continuous_relative_yaw;

        const float sin_yaw = arm_sin_f32(relative_yaw);
        const float cos_yaw = arm_cos_f32(relative_yaw);

        // Atlas uses a forward/right translation vector:
        //   command_x = forward, command_y = right.
        // For this axis convention, converting a vector from the gimbal frame
        // to the chassis frame uses the inverse-looking sign pattern below.
        // With relative_yaw = gimbal_world_yaw - chassis_world_yaw, composing
        // this transform with the chassis heading keeps a fixed operator
        // command fixed in the world/gimbal direction during SPIN.
        const float field_x = cos_yaw * command_x + sin_yaw * command_y;
        const float field_y = -sin_yaw * command_x + cos_yaw * command_y;
        g_telemetry.operator_x = command_x;
        g_telemetry.operator_y = command_y;
        g_telemetry.operator_turn = command_turn;
        g_telemetry.transformed_x = field_x;
        g_telemetry.transformed_y = field_y;

        float target_vx_command = 0.0f;
        float target_vy_command = 0.0f;
        float target_vt_command = 0.0f;

        switch (mode) {
            case REMOTE_MODE_MANUAL:
                target_vx_command = command_x;
                target_vy_command = command_y;
                target_vt_command =
                    ManualTurnCommand(command_x, command_y, command_turn,
                                      relative_yaw);
                g_telemetry.follow_error = 0.0f;
                break;

            case REMOTE_MODE_FOLLOW:
                target_vx_command = field_x;
                target_vy_command = field_y;
                target_vt_command =
                    FollowTurnCommand(continuous_relative_yaw);
                g_heading_hold_active = false;
                g_telemetry.heading_hold_active = false;
                break;

            case REMOTE_MODE_SPIN:
                target_vx_command = field_x;
                target_vy_command = field_y;
                LimitVectorMagnitude(target_vx_command, target_vy_command,
                                     kSpinMaxTranslationCommand);
                target_vt_command = SpinTurnCommand(command_turn);
                g_heading_hold_active = false;
                g_telemetry.heading_hold_active = false;
                g_telemetry.follow_error = 0.0f;
                break;

            case REMOTE_MODE_AUTOAIM:
            case REMOTE_MODE_AUTO:
            default:
                target_vx_command = field_x;
                target_vy_command = field_y;
                target_vt_command = 0.0f;
                g_heading_hold_active = false;
                g_telemetry.heading_hold_active = false;
                g_telemetry.follow_error = 0.0f;
                break;
        }

        const float target_vx =
            clip<float>(target_vx_command, -1.0f, 1.0f) * kMaxXySpeed;
        const float target_vy =
            clip<float>(target_vy_command, -1.0f, 1.0f) * kMaxXySpeed;
        const float target_vt =
            clip<float>(target_vt_command, -1.0f, 1.0f) * kMaxTurnSpeed;

        slew_vector_2d(target_vx, target_vy, kTranslationVectorStep,
                       g_smoothed_vx, g_smoothed_vy);
        g_telemetry.vx = g_smoothed_vx;
        g_telemetry.vy = g_smoothed_vy;
        g_telemetry.vt = g_ease_vt.Calc(target_vt);

        // g_telemetry.vx/vy remain logical forward/right values for UI and
        // diagnostics.  Convert only at the chassis-library boundary.
        MapLogicalToChassisLibrary(
            g_telemetry.vx, g_telemetry.vy,
            g_telemetry.library_x_speed, g_telemetry.library_y_speed);
        g_telemetry.frame_offset_deg = ChassisGetFrameOffsetDeg();
        g_chassis->SetSpeed(g_telemetry.library_x_speed,
                            g_telemetry.library_y_speed, g_telemetry.vt);

        g_telemetry.online = true;
        osDelay(5);
    }
}

bool ChassisForwardPowerLimit(bool enabled, uint8_t max_watt,
                              float buffer_joule, bool referee_online) {
    if (g_power_sender == nullptr) return false;
    // 0x72 keeps the legacy voltage slot reserved at 0.0 V. The lower board
    // owns chassis-bus voltage/current acquisition and never depends on an
    // upper-board ADC value for power control.
    g_power_sender->UpdatePowerLimitState(enabled, max_watt, 0.0f,
                                          buffer_joule, referee_online);
    return true;
}

bool ChassisForwardCurrentPower(float measured_power, float buffer_joule) {
    if (g_power_sender == nullptr) return false;
    g_power_sender->UpdateCurrentPower(measured_power, buffer_joule);
    return true;
}

ChassisTelemetry_t ChassisGetTelemetry() {
    return g_telemetry;
}

float ChassisGetTurnSpeed() {
    return g_telemetry.vt;
}
