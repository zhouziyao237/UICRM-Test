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
// Created by Administrator on 2025/8/13.
//

#include "MspSerial.h"

#include <string.h>

#include "Device/bsp_serial/DeviceSerial.h"
#include "Device/bsp_serial/bsp_Serial.h"
#include "MspProcessor.h"
#include "PG/msp.h"
#include "Target.h"
#include "bsp_os.h"

namespace RMFC {
    namespace MSP {

        static mspPort_t mspPorts[MAX_MSP_PORT_COUNT];
        static MspProcessor processor;

        void resetMspPort(mspPort_t* mspPortToReset, serialPort_t* serialPort) {
            *mspPortToReset = mspPort_t{};
            mspPortToReset->port = serialPort;
        }

        void mspSerialAllocatePorts(void) {
            uint8_t portIndex = 0;
            const serialPortConfig_t* portConfig = findSerialPortConfig(FUNCTION_MSP);
            while (portConfig && portIndex < MAX_MSP_PORT_COUNT) {
                mspPort_t* mspPort = &mspPorts[portIndex];

                if (mspPort->port) {
                    portIndex++;
                    continue;
                }

                portOptions_e options = SERIAL_NOT_INVERTED;

                if (mspConfig()->halfDuplex) {
                    options = static_cast<portOptions_e>(options | SERIAL_CHECK_TX);
                } else if ((portConfig->identifier >= SERIAL_PORT_USART1) &&
                           (portConfig->identifier <= SERIAL_PORT_USART8)) {
                    options = static_cast<portOptions_e>(options | SERIAL_CHECK_TX);
                }

                serialPort_t* serialPort =
                    openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL, NULL,
                                   baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, options);
                if (serialPort) {
                    resetMspPort(mspPort, serialPort);
                    portIndex++;
                }

                portConfig = findNextSerialPortConfig(FUNCTION_MSP);
            }
        }

        void mspSerialReleasePortIfAllocated(serialPort_t* serialPort) {
            for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                mspPort_t* candidateMspPort = &mspPorts[portIndex];
                if (candidateMspPort->port == serialPort) {
                    closeSerialPort(serialPort);
                    *candidateMspPort = mspPort_t{};
                }
            }
        }

        mspDescriptor_t getMspSerialPortDescriptor(const uint8_t portIdentifier) {
            for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                mspPort_t* candidateMspPort = &mspPorts[portIndex];
                if (candidateMspPort->port->identifier == portIdentifier) {
                    return candidateMspPort->descriptor;
                }
            }
            return -1;
        }

        static void mspEvaluateNonMspData(mspPort_t* mspPort, uint8_t receivedChar) {
#ifdef USE_CLI
            if (receivedChar == '#') {
                mspPort->msg.pendingRequest = MSP_PENDING_CLI;
                return;
            }
#endif

            if (receivedChar == serialConfig()->reboot_character) {
                mspPort->msg.pendingRequest = MSP_PENDING_BOOTLOADER_ROM;
                return;
            }
        }

        static void mspProcessPendingRequest(mspPort_t* mspPort) {
            // If no request is pending or 100ms guard time has not elapsed - do nothing
            if ((mspPort->msg.pendingRequest == MSP_PENDING_NONE) ||
                (bsp::GetHighresTickMicroSec() - mspPort->lastActivityMs < 100)) {
                return;
            }

            switch (mspPort->msg.pendingRequest) {
                case MSP_PENDING_BOOTLOADER_ROM:
                    // systemResetToBootloader();
                    break;

#ifdef USE_CLI
                case MSP_PENDING_CLI:
                    // cliEnter(mspPort->port);
                    break;
#endif

                default:
                    break;
            }
        }

        /*
         * Process MSP commands from serial ports configured as MSP ports.
         *
         * Called periodically by the scheduler.
         */
        void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData) {
            for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                mspPort_t* const mspPort = &mspPorts[portIndex];
                if (!mspPort->port) {
                    continue;
                }

                if (serialRxBytesWaiting(mspPort->port)) {
                    // There are bytes incoming - abort pending request
                    mspPort->lastActivityMs = bsp::GetHighresTickMilliSec();
                    mspPort->msg.pendingRequest = MSP_PENDING_NONE;

                    size_t len = serialRxBytesWaiting(mspPort->port);
                    if (len > 0) {
                        uint8_t buff[64] = {0};
                        len = std::min(len, sizeof(buff));

                        for (size_t i = 0; i < len; i++) {
                            uint8_t* dataPtr;
                            serialRead(mspPort->port, &dataPtr);
                            buff[i] = *dataPtr;
                        }

                        char* c = (char*)&buff[0];
                        while (len--) {
                            bool consumed =
                                processor.process(*c, mspPort->msg, mspPort->res, mspPort->port);

                            if (!consumed && evaluateNonMspData == MSP_EVALUATE_NON_MSP_DATA) {
                                mspEvaluateNonMspData(mspPort, *c);
                            }

                            c++;
                        }
                    }
                } else {
                    mspProcessPendingRequest(mspPort);
                }
            }
        }

        bool mspSerialWaiting(void) {
            for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                mspPort_t* const mspPort = &mspPorts[portIndex];
                if (!mspPort->port) {
                    continue;
                }

                if (serialRxBytesWaiting(mspPort->port)) {
                    return true;
                }
            }
            return false;
        }

        void mspSerialInit(void) {
            *mspPorts = mspPort_t{};
            mspSerialAllocatePorts();
        }

        int mspSerialPush(serialPortIdentifier_e portId, uint8_t cmd, uint8_t* data, int datalen) {
            UNUSED(cmd);
            int ret = 0;

            for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                mspPort_t* const mspPort = &mspPorts[portIndex];

                // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
                if (!mspPort->port
#ifndef USE_MSP_PUSH_OVER_VCP
                    || mspPort->port->identifier == SERIAL_PORT_USB_VCP
#endif
                    || (portId != SERIAL_PORT_ALL && mspPort->port->identifier != portId)) {
                    continue;
                }

                // 直接发送数据
                serialBeginWrite(mspPort->port);
                serialWriteBuf(mspPort->port, data, datalen);
                serialEndWrite(mspPort->port);

                ret = datalen;
            }
            return ret;
        }

        uint32_t mspSerialTxBytesFree(void) {
            uint32_t ret = UINT32_MAX;

            for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                mspPort_t* const mspPort = &mspPorts[portIndex];
                if (!mspPort->port) {
                    continue;
                }

                // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
                if (mspPort->port->identifier == SERIAL_PORT_USB_VCP) {
                    continue;
                }

                const uint32_t bytesFree = serialTxBytesFree(mspPort->port);
                if (bytesFree < ret) {
                    ret = bytesFree;
                }
            }

            return ret;
        }

    }  // namespace MSP
}  // namespace RMFC