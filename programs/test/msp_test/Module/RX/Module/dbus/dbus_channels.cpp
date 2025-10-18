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
#include "utils.h"

namespace RMFC {
    namespace RX {

        Client_ChannelsData_t* Client_ChannelsData = nullptr;

        uint8_t dbusChannelsDecode(rxRuntimeState_t* rxRuntimeState, const DbusFrame_t* DbusFrame) {
            uint16_t* dbusChannelData = rxRuntimeState->channelData;
            dbusChannelData[0] = DbusFrame->dbusChannels.ch0;
            dbusChannelData[1] = DbusFrame->dbusChannels.ch1;
            dbusChannelData[2] = DbusFrame->dbusChannels.ch2;
            dbusChannelData[3] = DbusFrame->dbusChannels.ch3;
            dbusChannelData[4] = DbusFrame->dbusChannels.ch4;
            dbusChannelData[5] = static_cast<switch_t>(DbusFrame->dbusChannels.swl);
            dbusChannelData[6] = static_cast<switch_t>(DbusFrame->dbusChannels.swr);

            dbusChannelData[7] = static_cast<uint16_t>(static_cast<uint16_t>(DbusFrame->dbusChannels.mouse.x) & 0xFFFF);
            dbusChannelData[8] = static_cast<uint16_t>(static_cast<uint16_t>(DbusFrame->dbusChannels.mouse.y) & 0xFFFF);
            dbusChannelData[9] = static_cast<uint16_t>(static_cast<uint16_t>(DbusFrame->dbusChannels.mouse.z) & 0xFFFF);
            dbusChannelData[10] = static_cast<uint16_t>(DbusFrame->dbusChannels.mouse.l);
            dbusChannelData[11] = static_cast<uint16_t>(DbusFrame->dbusChannels.mouse.r);

            uint16_t keyCode = DbusFrame->dbusChannels.keyboard.code;
            for (uint8_t i = 0; i < DBUS_MAX_KEYBOARD_CHANNEL; i++) {
                dbusChannelData[12 + i] = (keyCode >> i) & 0x1;
            }

            memcpy(&Client_ChannelsData->mouse, &DbusFrame->dbusChannels.mouse,
                   sizeof(mouseChannels_t));
            memcpy(&Client_ChannelsData->keyboard, &DbusFrame->dbusChannels.keyboard,
                   sizeof(keyboardChannels_t));

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
            UNUSED(rxRuntimeState);
            UNUSED(ch);

            switch (ch) {
                case 0 ...(DBUS_MAX_CHANNEL - 4):
                    channelData =
                        (5 * (float)(rxRuntimeState->channelData[ch] - RC_ROCKER_MID) / 8) + 1500;
                    channelData =
                        abs(channelData - 1500) <= RC_ROCKER_ZERO_DRIFT ? 1500 : channelData;
                    break;
                case 4:
                    channelData =
                        (5 * (float)(rxRuntimeState->channelData[ch] - RC_ROCKER_MID) / 8) + 1500;
                    channelData =
                        abs(channelData - 1500) <= RC_ROCKER_ZERO_DRIFT ? 1500 : channelData;
                    break;
                case (DBUS_MAX_CHANNEL - 2)...(DBUS_MAX_CHANNEL - 1):
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
                    break;
                case DBUS_MAX_CHANNEL ...(DBUS_MAX_CHANNEL + 2):
                    // 注意：这里原代码有副作用，修改了rxRuntimeState->channelData[ch]的值
                    // 在switch中不建议有副作用操作，建议重构
                    channelData = (40 * (float)rxRuntimeState->channelData[ch] / 8) + 1500;
                    break;
                case (DBUS_MAX_CHANNEL + 3)...(DBUS_CHANNEL - 1):
                    channelData = rxRuntimeState->channelData[ch] ? rxConfig()->maxcheck
                                                                  : rxConfig()->mincheck;
                    break;
                default:
                    break;
            }

            return channelData;
        }

        void dbusChannelsInit(const rxConfig_t* rxConfig, rxRuntimeState_t* rxRuntimeState) {
            rxRuntimeState->rcReadRawFn = dbusChannelsReadRawRC;

            if (Client_ChannelsData == nullptr) {
                Client_ChannelsData = new Client_ChannelsData_t;
            }

            for (uint8_t ch = 0; ch < DBUS_CHANNEL; ch++) {
                switch (ch) {
                    case 0 ...(DBUS_MAX_CHANNEL - 4):
                        rxRuntimeState->channelData[ch] = (16 * rxConfig->midrc) / 10 - 2400;
                        break;
                    case 4:
                        rxRuntimeState->channelData[ch] = (16 * rxConfig->midrc) / 10 - 2400;
                        break;
                    case (DBUS_MAX_CHANNEL - 2)...(DBUS_MAX_CHANNEL - 1):
                        rxRuntimeState->channelData[ch] = (16 * rxConfig->midrc) / 10 - 2400;
                        break;
                    case DBUS_MAX_CHANNEL ...(DBUS_MAX_CHANNEL + 2):
                        rxRuntimeState->channelData[ch] = (16 * rxConfig->midrc) / 10 - 2400;
                        break;
                    case (DBUS_MAX_CHANNEL + 3)...(DBUS_CHANNEL - 1):
                        // 处理 ch >= (DBUS_MAX_CHANNEL + DBUS_MAX_MOUSE_CHANNEL) 的情况
                        // todo: 默认没按下直接就是1000， 把0和1映射为1000与2000
                        rxRuntimeState->channelData[ch] = rxConfig->mincheck;
                        break;
                    default:
                        break;
                }
            }
        }

    }  // namespace RX
}  // namespace RMFC
