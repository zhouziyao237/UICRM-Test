// Copyright (c) 2025. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by Administrator on 2025/10/4.
//

#pragma once

#include <stdint.h>

#include "RX/rx.h"

#define DBUS_MAX_CHANNEL 7
#define DBUS_MAX_MOUSE_CHANNEL 5
#define DBUS_MAX_KEYBOARD_CHANNEL 16

#define DBUS_CHANNEL (DBUS_MAX_CHANNEL + DBUS_MAX_MOUSE_CHANNEL + DBUS_MAX_KEYBOARD_CHANNEL)

/* rocker range and deadzones */
#define RC_ROCKER_MID 1024
#define RC_ROCKER_ZERO_DRIFT 10  // Range of possible drift around initial position
// Range of possible drift around min or max position

#define MOUSE_XY_MAX 32767.0

namespace RMFC {
    namespace RX {

        typedef enum {
            UP = 1,
            DOWN = 2,
            MID = 3,
        } switch_t;

        typedef struct mouseChannels_s {
            /* mouse movement and button information */
            int16_t x;
            int16_t y;
            int16_t z;
            uint8_t l;
            uint8_t r;
        } __packed mouseChannels_t;

        typedef union keyboardChannels_s {
            uint16_t code;
            struct {
                /* keyboard key information */
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
        } __packed keyboardChannels_t;

        typedef struct dbusChannels_s {
            uint16_t ch0 : 11;  // S1*             *S2
            uint16_t ch1 : 11;  //   C3-^       ^-C1
            uint16_t ch2 : 11;  // C2-<   >+ -<   >+C0
            uint16_t ch3 : 11;  //     +v       v+
            /* left and right switch information */
            uint8_t swr : 2;
            uint8_t swl : 2;
            mouseChannels_t mouse;        // 鼠标数据
            keyboardChannels_t keyboard;  // 键盘数据
            uint16_t ch4 : 11;            // 波轮通道
        } __attribute__((__packed__)) dbusChannels_t;

        typedef struct DbusFrame_s {
            dbusChannels_t dbusChannels;
        } __attribute__((__packed__)) DbusFrame_t;

        uint8_t dbusChannelsDecode(rxRuntimeState_t* rxRuntimeState, const DbusFrame_t* DbusFrame);

        void dbusChannelsInit(const rxConfig_t* rxConfig, rxRuntimeState_t* rxRuntimeState);

    }  // namespace RX
}  // namespace RMFC
