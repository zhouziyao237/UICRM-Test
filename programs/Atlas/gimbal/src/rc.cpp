#include "rc.h"

#include <cmath>

#include "RCKEY.h"
#include "atlas_control_config.h"
#include "bsp_error_handler.h"
#include "dbus.h"
#include "main.h"
#include "utils.h"

osThreadId_t RCTaskHandle;

namespace {
constexpr float kGimbalStickGain = 0.06f;
constexpr float kGimbalMouseGain = 0.8f;

// Application-level deadbands. The DBUS driver already removes the first
// +/-10 raw counts. These normalized thresholds remove the remaining operator
// input noise before it reaches chassis/gimbal control.
constexpr float kStickDeadband = 0.04f;
constexpr float kTurnDeadband = 0.12f;
constexpr float kMouseDeadband = 0.01f;

constexpr uint8_t kTriggerKey = 0;
constexpr uint8_t kMouseFireKey = 1;
constexpr uint8_t kUiClearKey = 2;
constexpr rc_key_config_t kKeyConfigs[] = {
    {kTriggerKey, 0, 1, 500, 150},
    {kMouseFireKey, 0, 1, 500, 150},
    {kUiClearKey, 0, 1, 500, 200},
};

remote::DBUS* dbus = nullptr;
RemoteControl_t remote_data{};
bool was_online = false;

struct KeyboardMotionState {
    float forward = 0.0f;
    float lateral = 0.0f;
    float turn = 0.0f;
    uint32_t last_update_ms = 0U;
    bool initialized = false;
};

KeyboardMotionState keyboard_motion{};

void ResetKeyboardMotion() {
    keyboard_motion = {};
}

float MoveToward(float current, float target, float max_delta) {
    if (!(max_delta > 0.0f)) return current;
    if (current < target) return min(current + max_delta, target);
    if (current > target) return max(current - max_delta, target);
    return current;
}

float KeyboardDeltaTimeSeconds() {
    const uint32_t now_ms = HAL_GetTick();
    if (!keyboard_motion.initialized) {
        keyboard_motion.initialized = true;
        keyboard_motion.last_update_ms = now_ms;
        return 0.0f;
    }

    const uint32_t elapsed_ms = now_ms - keyboard_motion.last_update_ms;
    keyboard_motion.last_update_ms = now_ms;
    return static_cast<float>(min<uint32_t>(elapsed_ms, 20U)) * 0.001f;
}

bool AnyKeyboardMotionKeyDown(const remote::RemoteInput& input) {
    return input.KeyDown(atlas::controls::kMoveForwardKey) ||
           input.KeyDown(atlas::controls::kMoveBackwardKey) ||
           input.KeyDown(atlas::controls::kMoveLeftKey) ||
           input.KeyDown(atlas::controls::kMoveRightKey) ||
           input.KeyDown(atlas::controls::kTurnLeftKey) ||
           input.KeyDown(atlas::controls::kTurnRightKey);
}

void UpdateKeyboardMotion(const remote::RemoteInput& input, float& forward,
                          float& lateral, float& turn) {
    float wish_forward =
        (input.KeyDown(atlas::controls::kMoveForwardKey) ? 1.0f : 0.0f) -
        (input.KeyDown(atlas::controls::kMoveBackwardKey) ? 1.0f : 0.0f);
    float wish_lateral =
        (input.KeyDown(atlas::controls::kMoveRightKey) ? 1.0f : 0.0f) -
        (input.KeyDown(atlas::controls::kMoveLeftKey) ? 1.0f : 0.0f);

    // W+D must have the same magnitude as W. Normalize the digital wish
    // vector before applying cruise/sprint speed.
    const float wish_norm =
        std::sqrt(wish_forward * wish_forward + wish_lateral * wish_lateral);
    if (wish_norm > 1.0f) {
        wish_forward /= wish_norm;
        wish_lateral /= wish_norm;
    }

    // FPS-style speed modifiers. Crouch has priority when CTRL and SHIFT
    // are held together, preventing an accidental sprint while precision
    // movement is requested.
    const bool crouch = input.KeyDown(atlas::controls::kMoveCrouchKey);
    const bool sprint = input.KeyDown(atlas::controls::kMoveSprintKey);
    const float move_command =
        crouch ? atlas::controls::kKeyboardCrouchCommand
               : (sprint ? atlas::controls::kKeyboardSprintCommand
                         : atlas::controls::kKeyboardCruiseCommand);
    const float target_forward = wish_forward * move_command;
    const float target_lateral = wish_lateral * move_command;
    const float turn_command =
        crouch ? atlas::controls::kKeyboardCrouchTurnCommand
               : (sprint ? atlas::controls::kKeyboardSprintTurnCommand
                         : atlas::controls::kKeyboardCruiseTurnCommand);
    const float target_turn =
        ((input.KeyDown(atlas::controls::kTurnRightKey) ? 1.0f : 0.0f) -
         (input.KeyDown(atlas::controls::kTurnLeftKey) ? 1.0f : 0.0f)) *
        turn_command;

    const float dt = KeyboardDeltaTimeSeconds();

    const float target_norm =
        std::sqrt(target_forward * target_forward +
                  target_lateral * target_lateral);
    const float current_norm =
        std::sqrt(keyboard_motion.forward * keyboard_motion.forward +
                  keyboard_motion.lateral * keyboard_motion.lateral);
    const float direction_dot = keyboard_motion.forward * target_forward +
                                keyboard_motion.lateral * target_lateral;

    float move_rate = atlas::controls::kKeyboardMoveAcceleration;
    if (target_norm <= 0.0f) {
        move_rate = atlas::controls::kKeyboardMoveBraking;
    } else if (current_norm > 0.0f && direction_dot < 0.0f) {
        move_rate = atlas::controls::kKeyboardMoveReverseRate;
    } else if (target_norm < current_norm) {
        // SHIFT release and CTRL crouch are speed reductions, so use the
        // dedicated deceleration rate while preserving movement direction.
        move_rate = atlas::controls::kKeyboardMoveDeceleration;
    }
    slew_vector_2d(target_forward, target_lateral, move_rate * dt,
                   keyboard_motion.forward, keyboard_motion.lateral);

    float turn_rate = atlas::controls::kKeyboardTurnAcceleration;
    if (target_turn == 0.0f) {
        turn_rate = atlas::controls::kKeyboardTurnBraking;
    } else if (keyboard_motion.turn * target_turn < 0.0f) {
        turn_rate = atlas::controls::kKeyboardTurnReverseRate;
    } else if (std::fabs(target_turn) < std::fabs(keyboard_motion.turn)) {
        turn_rate = atlas::controls::kKeyboardTurnDeceleration;
    }
    keyboard_motion.turn =
        MoveToward(keyboard_motion.turn, target_turn, turn_rate * dt);

    forward = keyboard_motion.forward;
    lateral = keyboard_motion.lateral;
    turn = keyboard_motion.turn;
}

bool SwitchDown(const remote::RemoteInput& input,
                atlas::controls::SwitchBinding binding) {
    return binding.side != remote::SwitchSide::NONE &&
           input.HasSwitch(binding.side) &&
           input.GetSwitch(binding.side) == binding.position;
}

void ResetRemoteData(bool increment_generation) {
    ResetKeyboardMotion();
    const uint32_t generation =
        remote_data.generation + (increment_generation ? 1U : 0U);
    const uint32_t fire_sequence = remote_data.fire_sequence;
    const uint32_t ui_sequence = remote_data.ui_sequence;

    remote_data = {};
    remote_data.generation = generation;
    remote_data.physical_kill = true;
    remote_data.fire_sequence = fire_sequence;
    remote_data.ui_sequence = ui_sequence;
}

void OnKeyEvent(uint8_t channel, rc_key_mode_t mode) {
    if (mode != RC_KEY_CLICK && mode != RC_KEY_DOUBLE_CLICK &&
        mode != RC_KEY_LONG_PRESS) {
        return;
    }

    if (channel == kTriggerKey || channel == kMouseFireKey) {
        if (mode == RC_KEY_DOUBLE_CLICK) {
            remote_data.fire_command = REMOTE_FIRE_DOUBLE;
        } else if (mode == RC_KEY_LONG_PRESS) {
            remote_data.fire_command = REMOTE_FIRE_RAPID;
        } else {
            remote_data.fire_command = REMOTE_FIRE_SINGLE;
        }
        ++remote_data.fire_sequence;
        return;
    }

    if (channel == kUiClearKey) {
        remote_data.ui_command = mode == RC_KEY_LONG_PRESS
                                     ? REMOTE_UI_CLEAR_DIAGNOSTICS
                                     : REMOTE_UI_CLEAR_GRAPHICS;
        ++remote_data.ui_sequence;
    }
}

void ProcessTimedKeys(const remote::RemoteInput& input) {
    const bool trigger =
        SwitchDown(input, atlas::controls::kFireSwitch) ||
        input.KeyDown(atlas::controls::kFireKey);
    const bool mouse_fire =
        input.MouseButtonDown(atlas::controls::kFireMouseButton);
    const bool ui_clear = input.KeyDown(atlas::controls::kUiClearKey);

    rc_key_process(kTriggerKey, static_cast<uint16_t>(trigger));
    rc_key_process(kMouseFireKey, static_cast<uint16_t>(mouse_fire));
    rc_key_process(kUiClearKey, static_cast<uint16_t>(ui_clear));
    rc_key_process_events();
}

void MapFrame(const remote::RemoteInput& input) {
    RemoteControl_t next;
    next.online = true;
    next.generation = remote_data.generation;
    next.fire_command = remote_data.fire_command;
    next.fire_sequence = remote_data.fire_sequence;
    next.ui_command = remote_data.ui_command;
    next.ui_sequence = remote_data.ui_sequence;
    next.physical_kill =
        SwitchDown(input, atlas::controls::kPhysicalKillSwitch);

    next.cycle_mode =
        SwitchDown(input, atlas::controls::kCycleModeSwitch) ||
        input.KeyDown(atlas::controls::kCycleModeKey);
    next.manual_mode =
        input.KeyDown(atlas::controls::kManualModeKey);
    next.spin_mode = input.KeyDown(atlas::controls::kSpinModeKey);
    next.follow_mode =
        input.KeyDown(atlas::controls::kFollowModeKey);

    float forward =
        atlas::controls::kChassisForwardChannelSign *
        normalize_symmetric(
            input.GetChannel(atlas::controls::kChassisForwardChannel),
            remote::RemoteInput::CHANNEL_MAX, kStickDeadband);
    float lateral =
        atlas::controls::kChassisLateralChannelSign *
        normalize_symmetric(
            input.GetChannel(atlas::controls::kChassisLateralChannel),
            remote::RemoteInput::CHANNEL_MAX, kStickDeadband);
    float turn =
        atlas::controls::kChassisTurnChannelSign *
        normalize_symmetric(
            input.GetChannel(atlas::controls::kChassisTurnChannel),
            remote::RemoteInput::CHANNEL_MAX, kTurnDeadband);

    const bool analog_motion =
        forward != 0.0f || lateral != 0.0f || turn != 0.0f;
    if (analog_motion) {
        // Analog input stays linear and authoritative. Clear the digital ramp
        // so returning to the keyboard cannot revive a stale velocity.
        ResetKeyboardMotion();
    } else {
        UpdateKeyboardMotion(input, forward, lateral, turn);
        next.keyboard_motion =
            AnyKeyboardMotionKeyDown(input) || forward != 0.0f ||
            lateral != 0.0f || turn != 0.0f;
    }
    next.chassis_forward = forward;
    next.chassis_lateral = lateral;
    next.chassis_turn = turn;

    const float mouse_pitch = normalize_symmetric(
        input.GetMouseAxis(atlas::controls::kGimbalPitchMouseAxis),
        remote::RemoteInput::MOUSE_AXIS_MAX, kMouseDeadband);
    const float mouse_yaw = normalize_symmetric(
        input.GetMouseAxis(atlas::controls::kGimbalYawMouseAxis),
        remote::RemoteInput::MOUSE_AXIS_MAX, kMouseDeadband);

    if (mouse_pitch != 0.0f || mouse_yaw != 0.0f) {
        next.gimbal_pitch_delta = atlas::controls::kGimbalPitchMouseSign *
                                  mouse_pitch * kGimbalMouseGain;
        next.gimbal_yaw_delta = atlas::controls::kGimbalYawMouseSign *
                                mouse_yaw * kGimbalMouseGain;
    } else {
        const float pitch_stick = normalize_symmetric(
            input.GetChannel(atlas::controls::kGimbalPitchChannel),
            remote::RemoteInput::CHANNEL_MAX, kStickDeadband);
        const float yaw_stick = normalize_symmetric(
            input.GetChannel(atlas::controls::kGimbalYawChannel),
            remote::RemoteInput::CHANNEL_MAX, kStickDeadband);
        next.gimbal_pitch_delta =
            atlas::controls::kGimbalPitchChannelSign * pitch_stick *
            kGimbalStickGain;
        next.gimbal_yaw_delta =
            atlas::controls::kGimbalYawChannelSign * yaw_stick *
            kGimbalStickGain;
    }

    next.friction_toggle =
        SwitchDown(input, atlas::controls::kFrictionToggleSwitch) ||
        input.KeyDown(atlas::controls::kFrictionToggleKey);
    next.unload = input.KeyDown(atlas::controls::kUnloadKey);
    remote_data = next;
}

void HandleOnlineTransition(bool online) {
    if (online == was_online) return;
    was_online = online;
    ResetRemoteData(true);
    rc_key_reset_states();
}
}  // namespace

void InitRC() {
    dbus = new remote::DBUS(&huart3);
    RM_ASSERT_TRUE(dbus != nullptr, "DBUS init failed");

    rc_key_init();
    for (const auto& config : kKeyConfigs) {
        rc_key_add(config.channel, config);
    }
    rc_key_add_callback(OnKeyEvent);
    ResetRemoteData(false);
}

void RCTask(void* arg) {
    UNUSED(arg);
    while (true) {
        const bool online = dbus != nullptr && dbus->IsOnline();
        HandleOnlineTransition(online);

        if (online) {
            const remote::RemoteInput input = dbus->GetInput();
            MapFrame(input);
            ProcessTimedKeys(input);
        } else {
            ResetRemoteData(false);
            rc_key_reset_states();
        }
        osDelay(1);
    }
}

RemoteControl_t RCGetSnapshot() {
    return remote_data;
}

const char* RCSourceName() {
    return remote_data.online ? "DBUS/DT7" : "NONE";
}
