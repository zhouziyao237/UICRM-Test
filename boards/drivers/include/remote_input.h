/*###########################################################
 # Copyright (c) 2023-2026. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # Shared decoded operator input for DBUS and VTM.          #
 ###########################################################*/

#pragma once

#include <cstdint>

#include "main.h"

namespace remote {

enum class RemoteSource : uint8_t {
    NONE = 0U,
    DBUS,
    VTM,
};

typedef enum {
    UP = 1,
    DOWN = 2,
    MID = 3,
} switch_t;

// Both DBUS and VTM carry the same 16-bit keyboard bitmap.
typedef union {
    uint16_t code;
    struct {
        uint16_t W : 1;
        uint16_t S : 1;
        uint16_t A : 1;
        uint16_t D : 1;
        uint16_t SHIFT : 1;
        uint16_t CTRL : 1;
        uint16_t Q : 1;
        uint16_t E : 1;
        uint16_t R : 1;
        uint16_t F : 1;
        uint16_t G : 1;
        uint16_t Z : 1;
        uint16_t X : 1;
        uint16_t C : 1;
        uint16_t V : 1;
        uint16_t B : 1;
    } __packed bit;
} __packed keyboard_t;

enum class Channel : uint8_t {
    RIGHT_X = 0U,
    RIGHT_Y,
    LEFT_X,
    LEFT_Y,
    DIAL,
    TRIGGER,
};

enum class SwitchSide : uint8_t {
    NONE = 0U,
    LEFT,
    RIGHT,
};

enum class KeyboardKey : uint16_t {
    NONE = 0U,
    W = 1U << 0,
    S = 1U << 1,
    A = 1U << 2,
    D = 1U << 3,
    SHIFT = 1U << 4,
    CTRL = 1U << 5,
    Q = 1U << 6,
    E = 1U << 7,
    R = 1U << 8,
    F = 1U << 9,
    G = 1U << 10,
    Z = 1U << 11,
    X = 1U << 12,
    C = 1U << 13,
    V = 1U << 14,
    B = 1U << 15,
};

enum class MouseAxis : uint8_t {
    X = 0U,
    Y,
    Z,
};

enum class MouseButton : uint8_t {
    NONE = 0U,
    LEFT,
    RIGHT,
    MIDDLE,
};

enum class GearPosition : uint8_t {
    NONE = 0U,
    C,
    N,
    S,
};

enum class AuxButton : uint8_t {
    NONE = 0U,
    TRIGGER,
    PAUSE,
    CUSTOM_LEFT,
    CUSTOM_RIGHT,
};

// Receiver-independent decoded frame. Protocol-specific packet layouts stay in
// dbus_package.h and vtm_package.h; only decoded values enter this structure.
struct RemoteInput {
    RemoteSource source = RemoteSource::NONE;

    int16_t right_x = 0;
    int16_t right_y = 0;
    int16_t left_x = 0;
    int16_t left_y = 0;
    int16_t dial = 0;
    int16_t trigger = 0;

    bool has_left_switch = false;
    bool has_right_switch = false;
    switch_t left_switch = MID;
    switch_t right_switch = MID;

    GearPosition gear = GearPosition::NONE;
    bool pause_button = false;
    bool custom_left = false;
    bool custom_right = false;

    int16_t mouse_x = 0;
    int16_t mouse_y = 0;
    int16_t mouse_z = 0;
    bool mouse_left = false;
    bool mouse_right = false;
    bool mouse_middle = false;

    uint16_t keyboard_code = 0U;
    uint32_t timestamp = 0U;

    // DBUS and centered VTM rocker channels both use approximately +/-660.
    static constexpr int16_t CHANNEL_MAX = 660;
    static constexpr int16_t MOUSE_AXIS_MAX = 32767;

    constexpr int16_t GetChannel(Channel channel) const {
        switch (channel) {
            case Channel::RIGHT_X:
                return right_x;
            case Channel::RIGHT_Y:
                return right_y;
            case Channel::LEFT_X:
                return left_x;
            case Channel::LEFT_Y:
                return left_y;
            case Channel::DIAL:
                return dial;
            case Channel::TRIGGER:
                return trigger;
        }
        return 0;
    }

    constexpr bool HasSwitch(SwitchSide side) const {
        switch (side) {
            case SwitchSide::LEFT:
                return has_left_switch;
            case SwitchSide::RIGHT:
                return has_right_switch;
            case SwitchSide::NONE:
                return false;
        }
        return false;
    }

    constexpr switch_t GetSwitch(SwitchSide side) const {
        switch (side) {
            case SwitchSide::LEFT:
                return left_switch;
            case SwitchSide::RIGHT:
                return right_switch;
            case SwitchSide::NONE:
                return MID;
        }
        return MID;
    }

    constexpr bool KeyDown(KeyboardKey key) const {
        return key != KeyboardKey::NONE &&
               (keyboard_code & static_cast<uint16_t>(key)) != 0U;
    }

    constexpr int16_t GetMouseAxis(MouseAxis axis) const {
        switch (axis) {
            case MouseAxis::X:
                return mouse_x;
            case MouseAxis::Y:
                return mouse_y;
            case MouseAxis::Z:
                return mouse_z;
        }
        return 0;
    }

    constexpr bool MouseButtonDown(MouseButton button) const {
        switch (button) {
            case MouseButton::LEFT:
                return mouse_left;
            case MouseButton::RIGHT:
                return mouse_right;
            case MouseButton::MIDDLE:
                return mouse_middle;
            case MouseButton::NONE:
                return false;
        }
        return false;
    }

    constexpr bool AuxButtonDown(AuxButton button) const {
        switch (button) {
            case AuxButton::TRIGGER:
                return trigger > 0;
            case AuxButton::PAUSE:
                return pause_button;
            case AuxButton::CUSTOM_LEFT:
                return custom_left;
            case AuxButton::CUSTOM_RIGHT:
                return custom_right;
            case AuxButton::NONE:
                return false;
        }
        return false;
    }
};

}  // namespace remote
