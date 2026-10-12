/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "dbus.h"

#include <cmath>
#include <cstring>

#include "bsp_error_handler.h"
#include "utils.h"

/* rocker range and deadzones */
#define RC_ROCKER_MID 1024
#define RC_ROCKER_ZERO_DRIFT 10  // Range of possible drift around initial position
#define DBUS_ONLINE_TIMEOUT_MS 80U  // Fail-safe stale-input deadline
// Range of possible drift around min or max position

namespace remote {

    // helper struct for decoding raw bytes
    typedef struct {
        /* rocker channel information */
        uint16_t ch0 : 11;  // S1*             *S2
        uint16_t ch1 : 11;  //   C3-^       ^-C1
        uint16_t ch2 : 11;  // C2-<   >+ -<   >+C0
        uint16_t ch3 : 11;  //     +v       v+
        /* left and right switch information */
        uint8_t swr : 2;
        uint8_t swl : 2;
        /* mouse movement and button information */
        remote::mouse_t mouse;
        /* keyboard key information */
        remote::keyboard_t keyboard;
        uint16_t ch4 : 11;
    } __packed dbus_t;

    DBUS::DBUS(UART_HandleTypeDef* huart) : bsp::UART(huart) {
        SetupRx(sizeof(dbus_t) + 1);
        // DBUS frames arrive much faster than the generic 500 ms connection
        // timeout. Do not let stale operator commands survive a receiver/link loss.
        SetThreshold(DBUS_ONLINE_TIMEOUT_MS);
    }

    RemoteInput DBUS::GetInput() const {
        RemoteInput snapshot;
        uint32_t first_generation = 0U;
        uint32_t second_generation = 0U;
        do {
            first_generation = input_generation_;
            snapshot = input_frames_[active_input_index_];
            second_generation = input_generation_;
        } while (first_generation != second_generation);
        return snapshot;
    }

    void DBUS::RxCompleteCallback() {
        uint8_t* data = nullptr;
        // A partial/misaligned UART packet must not refresh the DBUS heartbeat.
        if (this->Read<true>(&data) != sizeof(dbus_t)) return;

        const dbus_t* repr = reinterpret_cast<const dbus_t*>(data);
        if (repr->swr < static_cast<uint8_t>(remote::UP) ||
            repr->swr > static_cast<uint8_t>(remote::MID) ||
            repr->swl < static_cast<uint8_t>(remote::UP) ||
            repr->swl > static_cast<uint8_t>(remote::MID)) {
            return;
        }

        RemoteInput next;
        next.source = RemoteSource::DBUS;
        next.right_x = apply_deadband<int16_t>(
            static_cast<int16_t>(repr->ch0 - RC_ROCKER_MID),
            RC_ROCKER_ZERO_DRIFT);
        next.right_y = apply_deadband<int16_t>(
            static_cast<int16_t>(repr->ch1 - RC_ROCKER_MID),
            RC_ROCKER_ZERO_DRIFT);
        next.left_x = apply_deadband<int16_t>(
            static_cast<int16_t>(repr->ch2 - RC_ROCKER_MID),
            RC_ROCKER_ZERO_DRIFT);
        next.left_y = apply_deadband<int16_t>(
            static_cast<int16_t>(repr->ch3 - RC_ROCKER_MID),
            RC_ROCKER_ZERO_DRIFT);
        next.dial = apply_deadband<int16_t>(
            static_cast<int16_t>(repr->ch4 - RC_ROCKER_MID),
            RC_ROCKER_ZERO_DRIFT);
        next.has_left_switch = true;
        next.has_right_switch = true;
        next.left_switch = static_cast<switch_t>(repr->swl);
        next.right_switch = static_cast<switch_t>(repr->swr);
        next.mouse_x = repr->mouse.x;
        next.mouse_y = repr->mouse.y;
        next.mouse_z = repr->mouse.z;
        next.mouse_left = repr->mouse.l != 0U;
        next.mouse_right = repr->mouse.r != 0U;
        next.mouse_middle = false;
        next.keyboard_code = repr->keyboard.code;

        Heartbeat();
        next.timestamp = GetLastUptime();

        // Publish a complete frame only after all fields have been decoded.
        const uint8_t next_index = active_input_index_ ^ 1U;
        input_frames_[next_index] = next;
        active_input_index_ = next_index;
        ++input_generation_;

        // Keep the original public fields for existing programs.
        ch0 = next.right_x;
        ch1 = next.right_y;
        ch2 = next.left_x;
        ch3 = next.left_y;
        ch4 = next.dial;
        swl = next.left_switch;
        swr = next.right_switch;
        mouse.x = next.mouse_x;
        mouse.y = next.mouse_y;
        mouse.z = next.mouse_z;
        mouse.l = next.mouse_left ? 1U : 0U;
        mouse.r = next.mouse_right ? 1U : 0U;
        keyboard.code = next.keyboard_code;
        timestamp = next.timestamp;
    }

} /* namespace remote */
