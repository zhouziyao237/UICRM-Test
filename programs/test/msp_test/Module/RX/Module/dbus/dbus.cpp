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

#include "dbus.h"

#include "Device/bsp_Serial/DeviceSerial.h"
#include "Device/bsp_Serial/bsp_serial.h"
#include "RX/rx.h"
#include "bsp_os.h"
#include "dbus_channels.h"
#include "pg/rx.h"
#include "utils.h"

#define DBUS_BAUDRATE 100000
#define DBUS_RX_REFRESH_RATE 14000  // 修改为14ms（14000微秒）

// todo： 有可能是因为接收超时导致后面的数据接收不到
// #define DBUS_TIME_NEEDED_PER_FRAME 3000
#define DBUS_TIME_NEEDED_PER_FRAME 5000

#define DBUS_FAST_BAUDRATE 200000
#define DBUS_FAST_RX_REFRESH_RATE 7000  // 快速模式下调整为7ms

#define DBUS_FRAME_SIZE (sizeof(DbusFrame_t) + 1)

#if !defined(DBUS_PORT_OPTIONS)
#define DBUS_PORT_OPTIONS (SERIAL_STOPBITS_1 | SERIAL_PARITY_EVEN)
#endif

#define DBUS_DIGITAL_CHANNEL_MIN 364
#define DBUS_DIGITAL_CHANNEL_MAX 1684

#define TX_LEN 1
#define RX_LEN 1

namespace RMFC {
    namespace RX {

        enum {
            DEBUG_DBUS_FRAME_FLAGS = 0,
            DEBUG_DBUS_STATE_FLAGS,
            DEBUG_DBUS_FRAME_TIME,
        };

        typedef struct dbusFrameData_s {
            DbusFrame_t frame;
            timeUs_t startAtUs;
            bool done;
            serialPort_t* DBusPort;
        } dbusFrameData_t;

        // Receive ISR callback
        static void dbusDataReceive(void* data) {
            dbusFrameData_t* dbusFrameData = reinterpret_cast<dbusFrameData_t*>(data);

            uint8_t* rxData;
            uint32_t len = serialRead<true>(dbusFrameData->DBusPort, &rxData);

            DbusFrame_t* dbusFrame = reinterpret_cast<DbusFrame_t*>(rxData);

            const timeUs_t nowUs = bsp::GetHighresTickMicroSec();

            const timeDelta_t sbusFrameTime = cmpTimeUs(nowUs, dbusFrameData->startAtUs);

            if (sbusFrameTime > (long)(DBUS_TIME_NEEDED_PER_FRAME + 500)) {
                if (len != sizeof(dbusChannels_t)) {
                    dbusFrameData->done = false;
                    return;
                }
                dbusFrameData->startAtUs = nowUs;
            }

            dbusFrameData->frame.dbusChannels = dbusFrame->dbusChannels;
            dbusFrameData->done = true;
        }

        static uint8_t dbusFrameStatus(rxRuntimeState_t* rxRuntimeState) {
            dbusFrameData_t* dbusFrameData =
                static_cast<dbusFrameData_t*>(rxRuntimeState->frameData);
            uint8_t frameStatus;

            if (!dbusFrameData->done) {
                return RX_FRAME_PENDING;
            }
            dbusFrameData->done = false;

            const timeUs_t nowUs = bsp::GetHighresTickMicroSec();
            const timeDelta_t frameTime = cmpTimeUs(nowUs, dbusFrameData->startAtUs);

            if (frameTime > DBUS_TIME_NEEDED_PER_FRAME + 500) {
                frameStatus = RX_FRAME_COMPLETE | RX_FRAME_DROPPED;
                return frameStatus;
            }

            frameStatus = dbusChannelsDecode(rxRuntimeState, &dbusFrameData->frame);

            if (!(frameStatus & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED))) {
                rxRuntimeState->lastRcFrameTimeUs = dbusFrameData->startAtUs;
            }

            return frameStatus;
        }

        bool dbusInit(const rxConfig_t* rxConfig, rxRuntimeState_t* rxRuntimeState) {
            static uint16_t dbusChannelData[DBUS_CHANNEL];
            static dbusFrameData_t dbusFrameData;
            static uint32_t dbusBaudRate;

            rxRuntimeState->channelData = dbusChannelData;
            rxRuntimeState->frameData = &dbusFrameData;
            dbusChannelsInit(rxConfig, rxRuntimeState);

            rxRuntimeState->channelCount = DBUS_CHANNEL;

            if (rxConfig->dbus_baud_fast) {
                dbusBaudRate = DBUS_FAST_BAUDRATE;
            } else {
                dbusBaudRate = DBUS_BAUDRATE;
            }

            rxRuntimeState->rcFrameStatusFn = dbusFrameStatus;
            rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;

            const serialPortConfig_t* portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
            if (!portConfig) {
                return false;
            }

            bool portShared = false;

            uartSetup_t uartSetup = {DBUS_FRAME_SIZE, true, TX_LEN, true};

            serialPort_t* sBusPort = openSerialPort(
                static_cast<serialPortIdentifier_e>(portConfig->identifier), FUNCTION_RX_SERIAL,
                static_cast<serial_rx_callback_t>(dbusDataReceive), &dbusFrameData, dbusBaudRate,
                portShared ? MODE_RXTX : MODE_RX,
                static_cast<portOptions_e>(DBUS_PORT_OPTIONS |
                                           (rxConfig->serialrx_inverted ? 0 : SERIAL_INVERTED) |
                                           (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)),
                &uartSetup);

            dbusFrameData.DBusPort = sBusPort;

            if (rxConfig->rssi_src_frame_errors) {
                rssiSource = RSSI_SOURCE_FRAME_ERRORS;
            }

            return sBusPort != NULL;
        }

    }  // namespace RX
}  // namespace RMFC