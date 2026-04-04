#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "dbus_package.h"

namespace atlas::controls {

using Channel = remote::Channel;
using KeyboardKey = remote::KeyboardKey;
using MouseAxis = remote::MouseAxis;
using MouseButton = remote::MouseButton;
using SwitchSide = remote::SwitchSide;

struct SwitchBinding {
    SwitchSide side;
    remote::switch_t position;
};

constexpr SwitchBinding DisabledSwitch() {
    return {SwitchSide::NONE, remote::MID};
}

// ---------------------------------------------------------------------------
// DT7/DR16 analog bindings. Channel selects the physical input; sign sets its
// direction. Use only +1.0f or -1.0f for signs.
// ---------------------------------------------------------------------------
constexpr Channel kChassisForwardChannel = Channel::RIGHT_Y;
constexpr Channel kChassisLateralChannel = Channel::RIGHT_X;
constexpr Channel kChassisTurnChannel = Channel::DIAL;
constexpr Channel kGimbalPitchChannel = Channel::LEFT_Y;
constexpr Channel kGimbalYawChannel = Channel::LEFT_X;

constexpr float kChassisForwardChannelSign = -1.0f;
constexpr float kChassisLateralChannelSign = 1.0f;  // positive = right
constexpr float kChassisTurnChannelSign = -1.0f;
constexpr float kGimbalPitchChannelSign = -1.0f;
constexpr float kGimbalYawChannelSign = 1.0f;

// Mouse X/Y/Z all come from the DBUS library. Select which axis controls each
// gimbal direction, then set its sign.
constexpr MouseAxis kGimbalPitchMouseAxis = MouseAxis::Y;
constexpr MouseAxis kGimbalYawMouseAxis = MouseAxis::X;
constexpr float kGimbalPitchMouseSign = -1.0f;
constexpr float kGimbalYawMouseSign = 1.0f;

// ---------------------------------------------------------------------------
// Keyboard movement bindings.
// ---------------------------------------------------------------------------
constexpr KeyboardKey kMoveForwardKey = KeyboardKey::W;
constexpr KeyboardKey kMoveBackwardKey = KeyboardKey::S;
constexpr KeyboardKey kMoveLeftKey = KeyboardKey::A;
constexpr KeyboardKey kMoveRightKey = KeyboardKey::D;
constexpr KeyboardKey kMoveSprintKey = KeyboardKey::SHIFT;
constexpr KeyboardKey kMoveCrouchKey = KeyboardKey::CTRL;
constexpr KeyboardKey kTurnLeftKey = KeyboardKey::Q;
constexpr KeyboardKey kTurnRightKey = KeyboardKey::E;

// Keyboard motion profile. WASD uses three FPS-style target speeds:
// CTRL crouch, normal cruise and SHIFT sprint. Every transition remains
// continuous. Raising the target uses acceleration; lowering a non-zero
// target uses deceleration; releasing movement uses braking; reversing uses
// the fastest reverse rate. Units are normalized chassis command per second.
constexpr float kKeyboardCrouchCommand = 0.25f;
constexpr float kKeyboardCruiseCommand = 0.50f;
constexpr float kKeyboardSprintCommand = 0.85f;
constexpr float kKeyboardMoveAcceleration = 0.75f;
constexpr float kKeyboardMoveDeceleration = 1.50f;
constexpr float kKeyboardMoveBraking = 2.00f;
constexpr float kKeyboardMoveReverseRate = 3.50f;

// Q/E follows the same crouch/cruise/sprint speed mode.
constexpr float kKeyboardCrouchTurnCommand = 0.20f;
constexpr float kKeyboardCruiseTurnCommand = 0.35f;
constexpr float kKeyboardSprintTurnCommand = 0.45f;
constexpr float kKeyboardTurnAcceleration = 1.10f;
constexpr float kKeyboardTurnDeceleration = 1.80f;
constexpr float kKeyboardTurnBraking = 2.50f;
constexpr float kKeyboardTurnReverseRate = 3.50f;

// ---------------------------------------------------------------------------
// Keyboard function bindings. KeyboardKey::NONE disables an optional binding.
// ---------------------------------------------------------------------------
constexpr KeyboardKey kCycleModeKey = KeyboardKey::G;
constexpr KeyboardKey kManualModeKey = KeyboardKey::B;
constexpr KeyboardKey kSpinModeKey = KeyboardKey::V;
constexpr KeyboardKey kFollowModeKey = KeyboardKey::C;
constexpr KeyboardKey kFrictionToggleKey = KeyboardKey::Z;
constexpr KeyboardKey kUnloadKey = KeyboardKey::R;
constexpr KeyboardKey kUiClearKey = KeyboardKey::X;
constexpr KeyboardKey kFireKey = KeyboardKey::NONE;

// ---------------------------------------------------------------------------
// Three-position switch bindings. SwitchSide::NONE disables an optional one.
// The physical KILL binding is mandatory.
// ---------------------------------------------------------------------------
constexpr SwitchBinding kPhysicalKillSwitch = {
    SwitchSide::RIGHT, remote::UP};
constexpr SwitchBinding kCycleModeSwitch = {
    SwitchSide::RIGHT, remote::DOWN};
constexpr SwitchBinding kFireSwitch = {
    SwitchSide::LEFT, remote::UP};
constexpr SwitchBinding kFrictionToggleSwitch = {
    SwitchSide::LEFT, remote::DOWN};

// Mouse fire binding. MouseButton::NONE disables mouse fire.
constexpr MouseButton kFireMouseButton = MouseButton::LEFT;

// ---------------------------------------------------------------------------
// Compile-time validation.
// ---------------------------------------------------------------------------
constexpr bool IsDirectionSign(float value) {
    return value == 1.0f || value == -1.0f;
}

constexpr bool IsSwitchPosition(remote::switch_t value) {
    return value == remote::UP || value == remote::MID ||
           value == remote::DOWN;
}

constexpr bool IsSwitchBindingValid(SwitchBinding binding) {
    if (binding.side == SwitchSide::NONE) {
        return binding.position == remote::MID;
    }
    return IsSwitchPosition(binding.position);
}

constexpr bool IsSwitchBindingEnabled(SwitchBinding binding) {
    return binding.side != SwitchSide::NONE;
}

constexpr bool SameEnabledSwitchBinding(SwitchBinding lhs,
                                        SwitchBinding rhs) {
    return IsSwitchBindingEnabled(lhs) && IsSwitchBindingEnabled(rhs) &&
           lhs.side == rhs.side && lhs.position == rhs.position;
}

template <std::size_t N>
constexpr bool KeyboardBindingsUnique(
    const std::array<KeyboardKey, N>& bindings) {
    for (std::size_t i = 0; i < N; ++i) {
        if (bindings[i] == KeyboardKey::NONE) continue;
        for (std::size_t j = i + 1; j < N; ++j) {
            if (bindings[i] == bindings[j]) return false;
        }
    }
    return true;
}

template <std::size_t N, std::size_t M>
constexpr bool KeyboardBindingGroupsDisjoint(
    const std::array<KeyboardKey, N>& lhs,
    const std::array<KeyboardKey, M>& rhs) {
    for (KeyboardKey left : lhs) {
        if (left == KeyboardKey::NONE) continue;
        for (KeyboardKey right : rhs) {
            if (left == right) return false;
        }
    }
    return true;
}

constexpr std::array<KeyboardKey, 8> kMovementKeys = {
    kMoveForwardKey, kMoveBackwardKey, kMoveLeftKey, kMoveRightKey,
    kMoveSprintKey, kMoveCrouchKey, kTurnLeftKey, kTurnRightKey};

constexpr std::array<KeyboardKey, 8> kFunctionKeys = {
    kCycleModeKey, kManualModeKey, kSpinModeKey, kFollowModeKey,
    kFrictionToggleKey, kUnloadKey, kUiClearKey, kFireKey};

static_assert(IsDirectionSign(kChassisForwardChannelSign));
static_assert(IsDirectionSign(kChassisLateralChannelSign));
static_assert(IsDirectionSign(kChassisTurnChannelSign));
static_assert(IsDirectionSign(kGimbalPitchChannelSign));
static_assert(IsDirectionSign(kGimbalYawChannelSign));
static_assert(IsDirectionSign(kGimbalPitchMouseSign));
static_assert(IsDirectionSign(kGimbalYawMouseSign));

static_assert(KeyboardBindingsUnique(kMovementKeys),
              "Atlas movement keys must be unique");
static_assert(KeyboardBindingsUnique(kFunctionKeys),
              "Atlas function keys must be unique unless disabled with NONE");
static_assert(KeyboardBindingGroupsDisjoint(kMovementKeys, kFunctionKeys),
              "Atlas movement and function keys must not overlap");

static_assert(IsSwitchBindingValid(kPhysicalKillSwitch));
static_assert(IsSwitchBindingValid(kCycleModeSwitch));
static_assert(IsSwitchBindingValid(kFireSwitch));
static_assert(IsSwitchBindingValid(kFrictionToggleSwitch));
static_assert(IsSwitchBindingEnabled(kPhysicalKillSwitch),
              "The physical KILL switch cannot be disabled");
static_assert(!SameEnabledSwitchBinding(kPhysicalKillSwitch,
                                        kCycleModeSwitch));
static_assert(!SameEnabledSwitchBinding(kPhysicalKillSwitch, kFireSwitch));
static_assert(!SameEnabledSwitchBinding(kPhysicalKillSwitch,
                                        kFrictionToggleSwitch));
static_assert(!SameEnabledSwitchBinding(kCycleModeSwitch, kFireSwitch));
static_assert(!SameEnabledSwitchBinding(kCycleModeSwitch,
                                        kFrictionToggleSwitch));
static_assert(!SameEnabledSwitchBinding(kFireSwitch,
                                        kFrictionToggleSwitch));

}  // namespace atlas::controls
