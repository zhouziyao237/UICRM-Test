/*
* Betaflight / Cleanflight Derivative Work
 * ----------------------------------------
 * Original code: Copyright (c) 2011-2025 Betaflight & Cleanflight contributors
 * Derivative work: Copyright (c) 2025 BNU-HKBU UIC RoboMaster
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"
#include "target.h"

#ifdef USE_SERIALRX_SBUS

#include "Device/bsp_Serial/DeviceSerial.h"
#include "Device/bsp_Serial/bsp_Serial.h"
#include "bsp_os.h"
// #include "build/debug.h"
#include "utils.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "pg/rx.h"
#include "rx/rx.h"
#include "sbus.h"
#include "sbus_channels.h"

/*
 * Observations
 *
 * FrSky X8R
 * time between frames: 6ms.
 * time to send frame: 3ms.
 *
 * Futaba R6208SB/R6303SB
 * time between frames: 11ms.
 * time to send frame: 3ms.
 */

#define SBUS_BAUDRATE 100000
#define SBUS_RX_REFRESH_RATE 11000
#define SBUS_TIME_NEEDED_PER_FRAME 3000

#define SBUS_FAST_BAUDRATE 200000
#define SBUS_FAST_RX_REFRESH_RATE 6000

#define SBUS_STATE_FAILSAFE (1 << 0)
#define SBUS_STATE_SIGNALLOSS (1 << 1)

#define SBUS_FRAME_SIZE (SBUS_CHANNEL_DATA_LENGTH + 2)

#define SBUS_FRAME_BEGIN_BYTE 0x0F

#if !defined(SBUS_PORT_OPTIONS)
#define SBUS_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)
#endif

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

namespace RMFC {
    namespace RX {
        enum {
            DEBUG_SBUS_FRAME_FLAGS = 0,
            DEBUG_SBUS_STATE_FLAGS,
            DEBUG_SBUS_FRAME_TIME,
        };

        struct sbusFrame_s {
            uint8_t syncByte;
            sbusChannels_t channels;
            /**
             * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value
             * indicates the telemetry byte that is sent after every 4th sbus frame.
             *
             * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
             * and
             * https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
             */
            uint8_t endByte;
        } __attribute__((__packed__));

        typedef union sbusFrame_u {
            uint8_t bytes[SBUS_FRAME_SIZE];
            struct sbusFrame_s frame;
        } sbusFrame_t;

        typedef struct sbusFrameData_s {
            sbusFrame_t frame;
            timeUs_t startAtUs;
            uint8_t position;
            bool done;
            serialPort_t* sBusPort;
        } sbusFrameData_t;

        // Receive ISR callback
        static void sbusDataReceive(void* data) {
            sbusFrameData_t* sbusFrameData = static_cast<sbusFrameData_t*>(data);

            // Check if the serial port is valid
            if (sbusFrameData->sBusPort == nullptr) {
                return;
            }

            // todo: 应该是 UART 的 ISR 清除就 SerialRead 接收 数据导致ISR卡了代码直接截断了
            uint8_t* rxData;
            uint32_t dataReceived = serialRead<true>(sbusFrameData->sBusPort, &rxData);

            // Check if we actually received data
            if (dataReceived > 0) {
                uint8_t c = rxData[0]; // Get the first byte of received data

                const timeUs_t nowUs = bsp::GetHighresTickMicroSec();

                const timeDelta_t sbusFrameTime = cmpTimeUs(nowUs, sbusFrameData->startAtUs);

                if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500)) {
                    sbusFrameData->position = 0;
                }

                if (sbusFrameData->position == 0) {
                    if (c != SBUS_FRAME_BEGIN_BYTE) {
                        return;
                    }
                    sbusFrameData->startAtUs = nowUs;
                }

                if (sbusFrameData->position < SBUS_FRAME_SIZE) {
                    sbusFrameData->frame.bytes[sbusFrameData->position++] = c;
                    if (sbusFrameData->position < SBUS_FRAME_SIZE) {
                        sbusFrameData->done = false;
                    } else {
                        sbusFrameData->done = true;
                        // DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_FRAME_TIME, sbusFrameTime);
                    }
                }
            }
        }

        static uint8_t sbusFrameStatus(rxRuntimeState_t* rxRuntimeState) {
            sbusFrameData_t* sbusFrameData = static_cast<sbusFrameData_t*>(rxRuntimeState->frameData);
            if (!sbusFrameData->done) {
                return RX_FRAME_PENDING;
            }
            sbusFrameData->done = false;

            // DEBUG_SET(DEBUG_SBUS, DEBUG_SBUS_FRAME_FLAGS,
            // sbusFrameData->frame.frame.channels.flags);

            const uint8_t frameStatus =
                sbusChannelsDecode(rxRuntimeState, &sbusFrameData->frame.frame.channels);

            if (!(frameStatus & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED))) {
                rxRuntimeState->lastRcFrameTimeUs = sbusFrameData->startAtUs;
            }

            return frameStatus;
        }

        bool sbusInit(const rxConfig_t* rxConfig, rxRuntimeState_t* rxRuntimeState) {
            static uint16_t sbusChannelData[SBUS_MAX_CHANNEL];
            static sbusFrameData_t sbusFrameData;
            static uint32_t sbusBaudRate;

            rxRuntimeState->channelData = sbusChannelData;
            rxRuntimeState->frameData = &sbusFrameData;
            sbusChannelsInit(rxConfig, rxRuntimeState);

            rxRuntimeState->channelCount = SBUS_MAX_CHANNEL;

            if (rxConfig->sbus_baud_fast) {
                sbusBaudRate = SBUS_FAST_BAUDRATE;
            } else {
                sbusBaudRate = SBUS_BAUDRATE;
            }

            rxRuntimeState->rcFrameStatusFn = sbusFrameStatus;
            rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;

            const serialPortConfig_t* portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
            if (!portConfig) {
                return false;
            }

#ifdef USE_TELEMETRY
            bool portShared =
                telemetryCheckRxPortShared(portConfig, rxRuntimeState->serialrxProvider);
#else
            bool portShared = false;
#endif

            serialPort_t* sBusPort = openSerialPort(
                static_cast<serialPortIdentifier_e>(portConfig->identifier),
                FUNCTION_RX_SERIAL,
                static_cast<serial_rx_callback_t>(sbusDataReceive),
                &sbusFrameData,
                sbusBaudRate, portShared ? MODE_RXTX : MODE_RX,
                static_cast<portOptions_e>(SBUS_PORT_OPTIONS |
                                           (rxConfig->serialrx_inverted ? 0 : SERIAL_INVERTED) |
                                           (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)));

            // Store the serial port reference for use in the callback
            sbusFrameData.sBusPort = sBusPort;

            if (rxConfig->rssi_src_frame_errors) {
                rssiSource = RSSI_SOURCE_FRAME_ERRORS;
            }

#ifdef USE_TELEMETRY
            if (portShared) {
                telemetrySharedPort = sBusPort;
            }
#endif

            return sBusPort != NULL;
        }
#endif
    }
}