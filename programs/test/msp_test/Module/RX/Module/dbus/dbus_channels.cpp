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

#include "dbus_channels.h"

#include "RX/rx.h"
#include "string.h"

namespace RMFC {
    namespace RX {

        uint8_t dbusChannelsDecode(rxRuntimeState_t* rxRuntimeState, const DbusFrame_t* DbusFrame) {
            uint16_t* dbusChannelData = rxRuntimeState->channelData;
            dbusChannelData[0] = DbusFrame->channels.ch0;
            dbusChannelData[1] = DbusFrame->channels.ch1;
            dbusChannelData[2] = DbusFrame->channels.ch2;
            dbusChannelData[3] = DbusFrame->channels.ch3;
            dbusChannelData[4] = DbusFrame->ch4;
            dbusChannelData[5] = static_cast<switch_t>(DbusFrame->channels.swl);
            dbusChannelData[6] = static_cast<switch_t>(DbusFrame->channels.swr);

            // memcpy(&DbusFrame->mouse, &mouse, sizeof(mouseChannels_t));
            // memcpy(&DbusFrame->keyboard, &keyboard, sizeof(keyboardChannels_t));

            // if (DbusFrame->flags & DBUS_FLAG_FAILSAFE_ACTIVE) {
            //     // internal failsafe enabled and rx failsafe flag set
            //     // RX *should* still be sending valid channel data (repeated), so use it.
            //     return RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
            // }

            // if (DbusFrame->flags & DBUS_FLAG_SIGNAL_LOSS) {
            //     // The received data is a repeat of the last valid data so can be considered
            //     // complete.
            //     return RX_FRAME_COMPLETE | RX_FRAME_DROPPED;
            // }

            return RX_FRAME_COMPLETE;
        }

        static float dbusChannelsReadRawRC(const rxRuntimeState_t* rxRuntimeState, uint8_t ch) {
            uint16_t channelData = -1;

            if (ch <= DBUS_MAX_CHANNEL - 3) {
                channelData = (5 * (float)rxRuntimeState->channelData[ch] / 8) + 1440;
            } else if (ch <= DBUS_MAX_CHANNEL - 2 && ch <= DBUS_MAX_CHANNEL - 1) {
                switch (rxRuntimeState->channelData[ch]) {
                    case UP:
                        channelData = rxConfig()->maxcheck;
                        break;
                    case MID:
                        channelData = rxConfig()->midrc;
                        break;
                    case DOWN:
                        channelData = rxConfig()->mincheck;
                        break;
                    default:
                        channelData = -1;
                        break;
                }
            }

            if (ch >= DBUS_MAX_CHANNEL && ch <= DBUS_MAX_CHANNEL + 2) {
                rxRuntimeState->channelData[ch] =
                    (5 * (float)rxRuntimeState->channelData[ch] / 8) + 1500;
            } else if (ch >= DBUS_MAX_CHANNEL + DBUS_MAX_MOUSE_CHANNEL - 2 &&
                       ch <= DBUS_MAX_CHANNEL + DBUS_MAX_MOUSE_CHANNEL - 1) {
                channelData = rxRuntimeState->channelData[ch] ? 2000 : 1000;
            }

            if (ch >= DBUS_MAX_CHANNEL + DBUS_MAX_MOUSE_CHANNEL) {
                channelData = rxRuntimeState->channelData[ch] ? 2000 : 1000;
            }

            return channelData;
        }

#define RC_PWM_MAX rxConfig->maxcheck
#define RC_PWM_MIN rxConfig->mincheck
#define RC_PWM_MID rxConfig->midrc

        void dbusChannelsInit(const rxConfig_t* rxConfig, rxRuntimeState_t* rxRuntimeState) {
            rxRuntimeState->rcReadRawFn = dbusChannelsReadRawRC;
            for (uint8_t ch = 0; ch < DBUS_CHANNEL; ch++) {
                if (ch <= DBUS_MAX_CHANNEL - 3) {
                    rxRuntimeState->channelData[ch] = (16 * RC_PWM_MID) / 10 - 1376;
                } else if (ch <= DBUS_MAX_CHANNEL - 2 && ch <= DBUS_MAX_CHANNEL - 1) {
                    rxRuntimeState->channelData[ch] = RC_PWM_MID;
                }

                if (ch >= DBUS_MAX_CHANNEL && ch <= DBUS_MAX_CHANNEL + 2) {
                    rxRuntimeState->channelData[ch] = (16 * RC_PWM_MID) / 10 - 2400;
                } else if (ch >= DBUS_MAX_CHANNEL + DBUS_MAX_MOUSE_CHANNEL - 2 &&
                           ch <= DBUS_MAX_CHANNEL + DBUS_MAX_MOUSE_CHANNEL - 1) {
                    // todo: 默认没按下直接就是1000， 把0和1映射为1000与2000
                    rxRuntimeState->channelData[ch] = RC_PWM_MIN;
                }

                if (ch >= DBUS_MAX_CHANNEL + DBUS_MAX_MOUSE_CHANNEL) {
                    // todo: 默认没按下直接就是1000， 把0和1映射为1000与2000
                    rxRuntimeState->channelData[ch] = RC_PWM_MIN;
                }
            }
        }

    }  // namespace RX
}  // namespace RMFC
