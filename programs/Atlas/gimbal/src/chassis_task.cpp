#include "chassis_task.h"

#include <cmath>

#include "MotorCanBase.h"
#include "atlas_config.h"
#include "bsp_batteryvol.h"
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
bsp::BatteryVol* g_battery = nullptr;

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
uint32_t g_last_power_update_ms = 0U;
bool g_power_update_initialized = false;
constexpr uint32_t kPowerUpdatePeriodMs = 100U;

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

void UpdateBatteryAndPowerLimit(bool force = false) {
    const uint32_t now_ms = HAL_GetTick();
    if (!force && g_power_update_initialized &&
        static_cast<uint32_t>(now_ms - g_last_power_update_ms) <
            kPowerUpdatePeriodMs) {
        return;
    }
    g_last_power_update_ms = now_ms;
    g_power_update_initialized = true;

    // Start from the formal safety state.  The optional no-referee bench
    // values are selected only under ATLAS_ENABLE_NO_REFEREE_POWER_TEST.
    uint8_t max_watt = kInvalidRefereePowerLimitW;
    float buffer_joule = kInvalidRefereeBufferJoule;
    float measured_power = -1.0f;
    bool referee_source_active = false;
    bool referee_limit_valid = false;
    bool local_test_active = false;
    bool referee_failsafe = true;

#ifdef HAS_REFEREE
    const RefereeSnapshot_t referee = RefereeGetSnapshot();
    if (referee.online) {
        // Keep competition limits authoritative even when the optional measured
        // power sample is temporarily invalid.  Falling back to the 100 W bench
        // setting while a level-1 infantry limit is 45/60 W would be unsafe.
        referee_source_active = true;
        referee_limit_valid =
            referee.chassis_power_limit > 0U &&
            static_cast<float>(referee.chassis_power_limit) <=
                kMaxValidRefereePowerW &&
            referee.chassis_power_buffer <= 60U;

        if (referee_limit_valid) {
            max_watt = static_cast<uint8_t>(referee.chassis_power_limit);
            buffer_joule = static_cast<float>(referee.chassis_power_buffer);
            referee_failsafe = false;
            if (std::isfinite(referee.chassis_power) &&
                referee.chassis_power >= 0.0f) {
                measured_power = referee.chassis_power;
            }
        } else {
            // Online-but-invalid data is a competition fail-safe, not a bench
            // fallback.  Use the minimum 2026 infantry power and no buffer.
            max_watt = kInvalidRefereePowerLimitW;
            buffer_joule = kInvalidRefereeBufferJoule;
            measured_power = -1.0f;
            referee_failsafe = true;
        }
    }
#endif

#if ATLAS_ENABLE_NO_REFEREE_POWER_TEST
    // Explicit bench-only mode.  An online referee, including its invalid-data
    // fail-safe state, always has priority over this test source.
    if (!referee_source_active) {
        max_watt = kLocalChassisPowerLimitW;
        buffer_joule = kLocalChassisBufferJoule;
        measured_power = -1.0f;
        local_test_active = true;
        referee_failsafe = false;
    }
#endif

    g_telemetry.battery_voltage =
        g_battery ? g_battery->GetBatteryVol() : 0.0f;
    g_telemetry.battery_percent =
        g_battery ? g_battery->GetBatteryPercentage() : 0.0f;
    g_telemetry.power_from_referee = referee_limit_valid;
    g_telemetry.power_local_test = local_test_active;
    g_telemetry.power_failsafe = referee_failsafe;
    g_telemetry.power_limit_w = static_cast<float>(max_watt);
    g_telemetry.power_buffer_joule = buffer_joule;
    g_telemetry.measured_power_w = measured_power;

    g_chassis->UpdatePower(true, max_watt, g_telemetry.battery_voltage,
                           buffer_joule, measured_power,
                           referee_source_active);
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
    g_battery =
        new bsp::BatteryVol(&hadc3, ADC_CHANNEL_8, 1, ADC_SAMPLETIME_3CYCLES);
    g_can_bridge = new communication::CanBridge(atlas_can1, 0x51);
    g_chassis = new control::ChassisCanBridgeSender(g_can_bridge, 0x52);
    g_chassis->SetChassisRegId(0x70, 0x71, 0x72, 0x73);
    g_chassis->Disable();

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
        if (!RobotCanRun() || !ChassisPowerAllowed()) {
            DisableOutput();
            g_previous_mode = REMOTE_MODE_KILL;
            osDelay(5);
            continue;
        }

        if (!g_telemetry.output_enabled) {
            g_chassis->Enable();
            ResetMotion();
            UpdateBatteryAndPowerLimit(true);
            g_telemetry.output_enabled = true;
            g_previous_mode = remote_mode;
        }

        float command_x = 0.0f;
        float command_y = 0.0f;
        float command_turn = 0.0f;
        ReadOperatorCommand(command_x, command_y, command_turn);

        // GimbalGetRelativeYaw() is a logical angle: positive has the same
        // meaning as positive operator yaw, independent of yaw motor mounting.
        const float relative_yaw = WrapPi(GimbalGetRelativeYaw());
        HandleModeTransition(remote_mode, command_turn, relative_yaw);
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

        switch (remote_mode) {
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

        UpdateBatteryAndPowerLimit();
        g_telemetry.online = true;
        osDelay(5);
    }
}

ChassisTelemetry_t ChassisGetTelemetry() {
    return g_telemetry;
}

float ChassisGetTurnSpeed() {
    return g_telemetry.vt;
}
