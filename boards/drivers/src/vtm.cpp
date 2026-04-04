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

#include "vtm.h"

#include <cmath>
#include <cstring>

#include "bsp_error_handler.h"
#include "crc_check.h"
#include "cmsis_os2.h"
#include "vtm_package.h"
#include "bsp_thread.h"
#include "FreeRTOS.h"
#include "utils.h"
#include "bsp_os.h"

namespace remote {

    RemoteInput VTM::GetInput() const {
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

    VTM::VTM(bsp::UART* uart) {
        if (!uart) {
            RM_ASSERT_TRUE(false, "VTM is NULL");
            return;
        }
        uart_ = uart;
        uart_->RegisterCallback(RxCallback, this);

        vtm_start_flag = true;
    }

    void VTM::RxCallback(void* args) {
        VTM* self = static_cast<VTM*>(args);
        if (self == nullptr || self->uart_ == nullptr || !self->vtm_start_flag)
            return;

        self->vtm_start_flag = false;
        package_t package = {};
        package.length = self->uart_->Read<true>(&package.data);

        bool valid_frame_received = false;
        if (package.data != nullptr && package.length >= PACKET_SIZE) {
            int offset = 0;
            while (offset + PACKET_SIZE <= package.length) {
                if (package.data[offset] != VTM_FRAME_HEADER_1 ||
                    package.data[offset + 1] != VTM_FRAME_HEADER_2) {
                    ++offset;
                    continue;
                }

                if (verify_crc16_check_sum(package.data + offset, PACKET_SIZE)) {
                    self->VTMupdata(package.data + offset);
                    valid_frame_received = true;
                    offset += PACKET_SIZE;
                } else {
                    ++offset;
                }
            }
        }

        if (valid_frame_received)
            self->Heartbeat();
        self->vtm_start_flag = true;
    }

    void VTM::VTMupdata(uint8_t* data) {
        const vtm_raw_t* repr = reinterpret_cast<const vtm_raw_t*>(data);

        if (repr->header1 != VTM_FRAME_HEADER_1 ||
            repr->header2 != VTM_FRAME_HEADER_2 ||
            !verify_crc16_check_sum(data, PACKET_SIZE)) {
            return;
        }

        RemoteInput next;
        next.source = RemoteSource::VTM;
        next.right_x = apply_deadband<int16_t>(
            static_cast<int16_t>(repr->ch0 - VTM_ROCKER_MID),
            VTM_ROCKER_ZERO_DRIFT);
        next.right_y = apply_deadband<int16_t>(
            static_cast<int16_t>(repr->ch1 - VTM_ROCKER_MID),
            VTM_ROCKER_ZERO_DRIFT);

        // VTM packet order is ch2=left Y and ch3=left X. The common frame
        // exposes semantic axes, so applications do not need protocol-specific
        // channel swaps.
        next.left_y = apply_deadband<int16_t>(
            static_cast<int16_t>(repr->ch2 - VTM_ROCKER_MID),
            VTM_ROCKER_ZERO_DRIFT);
        next.left_x = apply_deadband<int16_t>(
            static_cast<int16_t>(repr->ch3 - VTM_ROCKER_MID),
            VTM_ROCKER_ZERO_DRIFT);
        next.dial = apply_deadband<int16_t>(
            static_cast<int16_t>(repr->ch4 - VTM_ROCKER_MID),
            VTM_ROCKER_ZERO_DRIFT);
        next.trigger = repr->ch5 != 0U ? RemoteInput::CHANNEL_MAX : 0;

        switch (static_cast<vtm_gear_t>(repr->gear_switch)) {
            case VTM_GEAR_C:
                next.gear = GearPosition::C;
                break;
            case VTM_GEAR_N:
                next.gear = GearPosition::N;
                break;
            case VTM_GEAR_S:
                next.gear = GearPosition::S;
                break;
            default:
                next.gear = GearPosition::NONE;
                break;
        }
        next.pause_button = repr->pause_button != 0U;
        next.custom_left = repr->custom_left != 0U;
        next.custom_right = repr->custom_right != 0U;

        next.mouse_x = repr->mouse.x;
        next.mouse_y = repr->mouse.y;
        next.mouse_z = repr->mouse.z;
        next.mouse_left = repr->mouse.l != 0U;
        next.mouse_right = repr->mouse.r != 0U;
        next.mouse_middle = repr->mouse.m != 0U;
        next.keyboard_code = repr->keyboard.code;
        next.timestamp = bsp::GetHighresTickMilliSec();

        const uint8_t next_index = active_input_index_ ^ 1U;
        input_frames_[next_index] = next;
        active_input_index_ = next_index;
        ++input_generation_;

        // Keep the original VTM fields for existing programs.
        ch0 = next.right_x;
        ch1 = next.right_y;
        ch2 = next.left_y;
        ch3 = next.left_x;
        ch4 = next.dial;
        ch5 = repr->ch5 != 0U ? 1 : 0;
        gear_switch = static_cast<vtm_gear_t>(repr->gear_switch);
        pause_button = next.pause_button;
        custom_left = next.custom_left;
        custom_right = next.custom_right;
        memcpy(&mouse, &repr->mouse, sizeof(vtm_mouse_t));
        keyboard.code = next.keyboard_code;
        timestamp = next.timestamp;
    }

} /* namespace remote */
