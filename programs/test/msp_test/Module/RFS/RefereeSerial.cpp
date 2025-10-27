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
// Created by Administrator on 2025/8/20.
//

#include "RefereeSerial.h"

#include "OSD.h"
#include "PG/Referee.h"
#include "target.h"
#include "bsp_os.h"
#include "cmsis_os.h"

namespace RMFC {
    namespace RFS {

        serialPort_t* FindRefereeProtocol(RefereeProtConfig_t ProtConfig) {
            // todo: 没写FEATURE_RFS_MSP的设置，写完再开
            // if (!featureIsEnabled(ProtConfig.features)) {
            //     return false;
            // }

            const serialPortConfig_t* port = findSerialPortConfig(ProtConfig.serialPortFunction);
            if (port == nullptr) {
                return nullptr;
            }

            portOptions_e options = SERIAL_NOT_INVERTED;

            if ((port->identifier >= SERIAL_PORT_USART1) &&
                (port->identifier <= SERIAL_PORT_USART8)) {
                options = static_cast<portOptions_e>(options | SERIAL_CHECK_TX);
            }

            serialPort_t* Prot = openSerialPort(
                static_cast<serialPortIdentifier_e>(port->identifier),
                ProtConfig.serialPortFunction, NULL, NULL,
                baudRates[getBaudrateIndexByFunction(port, ProtConfig.baudRatesIndex)], MODE_RXTX,
                options);

            // InitTask();

            return Prot;
        }

// ---------------------------------------------- 分割线 ----------------------------------------------

        RefereePort_t RefereePort[MAX_RFS_PORT_COUNT];

        void resetRefereePort(RefereePort_t* RFSPortToReset, serialPort_t* serialPort) {
            *RFSPortToReset = RefereePort_t{};
            RFSPortToReset->port = serialPort;
        }

        void RefereeSerialReleasePortIfAllocated(serialPort_t* serialPort) {
            for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                RefereePort_t* candidateRefereePort = &RefereePort[portIndex];
                if (candidateRefereePort->port == serialPort) {
                    closeSerialPort(serialPort);
                    *candidateRefereePort = RefereePort_t{};
                }
            }
        }

        void RefereeSerialAllocatePorts(void) {
            uint8_t portIndex = 0;
            const serialPortConfig_t* portConfig = findSerialPortConfig(FUNCTION_RefereeHost);
            // todo:允许最大MAX_MSP_PORT_COUNT个msp端口，实际上默认分配N个，还有MAX_MSP_PORT_COUNT-N个是在地面站配置才生效
            while (portConfig && portIndex < MAX_MSP_PORT_COUNT) {
                RefereePort_t* RFSPort = &RefereePort[portIndex];

                if (RFSPort->port) {
                    portIndex++;
                    continue;
                }

                portOptions_e options = SERIAL_NOT_INVERTED;

                if (RefereeConfig()->halfDuplex) {
                    options = static_cast<portOptions_e>(options | SERIAL_CHECK_TX);
                } else if ((portConfig->identifier >= SERIAL_PORT_USART1) &&
                           (portConfig->identifier <= SERIAL_PORT_USART8)) {
                    options = static_cast<portOptions_e>(options | SERIAL_CHECK_TX);
                           }

                serialPort_t* serialPort = openSerialPort(
                    static_cast<serialPortIdentifier_e>(portConfig->identifier),
                    FUNCTION_RefereeHost,
                    NULL,
                    NULL,
                    baudRates[portConfig->referee_baudrateIndex],
                    MODE_RXTX,
                    options);

                if (serialPort) {
                    resetRefereePort(RFSPort, serialPort);
                    portIndex++;
                }

                RFSPort->Referee_protocol = new RefereeProtocol(serialPort, false);
                RFSPort->UI = new UserInterface(serialPort, RFSPort->Referee_protocol);

                portConfig = findNextSerialPortConfig(FUNCTION_RefereeHost);
            }
        }

        refereeDescriptor_t getRefereeSerialPortDescriptor(const uint8_t portIdentifier) {
            for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                RefereePort_t* candidateRefereePort = &RefereePort[portIndex];
                if (candidateRefereePort->port->identifier == portIdentifier) {
                    return candidateRefereePort->descriptor;
                }
            }
            return -1;
        }

        /*
         * Process Referee commands from serial ports configured as Referee ports.
         *
         * Called periodically by the scheduler.
         */
        void RefereeSerialProcess() {
            for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                RefereePort_t* const RFSPort = &RefereePort[portIndex];
                if (!RFSPort->port) {
                    continue;
                }

                // There are bytes incoming - abort pending request
                RFSPort->lastActivityMs = bsp::GetHighresTickMilliSec();
                RFSPort->package.rwstatus = Read_Status;

                uint32_t len = serialRead(RFSPort->port, reinterpret_cast<uint8_t**>(RFSPort->data));
                if (len > 0) {
                    char* str = reinterpret_cast<char*>(RFSPort->data);
                    for (uint32_t i = 0; i < len; i++) {
                        RFSPort->Referee_protocol->ProcessReceive(*str, RFSPort->package);
                        str++;
                    }
                }
            }
        }

        bool RefereeSerialWaiting(void) {
            for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                RefereePort_t* const RFSPort = &RefereePort[portIndex];
                if (!RFSPort->port) {
                    continue;
                }

                if (serialRxBytesWaiting(RFSPort->port)) {
                    return true;
                }
            }
            return false;
        }

        void RefereeSerialInit(void) {
            // todo: 没写FEATURE_REF_PC_MSP的设置，写完再开
            // if (!featureIsEnabled(FEATURE_REF_PC_MSP)) {
            //     return false;
            // }
            *RefereePort = RefereePort_t{};
            RefereeSerialAllocatePorts();
            print("[INFO] Referee Init!");
        }

        int RefereeSerialPush(serialPortIdentifier_e portId, uint8_t cmd, uint8_t* data, int datalen) {
            UNUSED(cmd);
            int ret = 0;

            for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                RefereePort_t* const RFSPort = &RefereePort[portIndex];

                // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
                if (!RFSPort->port
#ifndef USE_MSP_PUSH_OVER_VCP
                    || RFSPort->port->identifier == SERIAL_PORT_USB_VCP
#endif
                    || (portId != SERIAL_PORT_ALL && RFSPort->port->identifier != portId)) {
                    continue;
                }

                // 直接发送数据
                serialBeginWrite(RFSPort->port);
                serialWriteBuf(RFSPort->port, data, datalen);
                serialEndWrite(RFSPort->port);

                ret = datalen;
            }
            return ret;
        }

        uint32_t RefereeSerialTxBytesFree(void) {
            uint32_t ret = UINT32_MAX;

            for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                RefereePort_t* const RFSPort = &RefereePort[portIndex];
                if (!RFSPort->port) {
                    continue;
                }

                // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
                if (RFSPort->port->identifier == SERIAL_PORT_USB_VCP) {
                    continue;
                }

                const uint32_t bytesFree = serialTxBytesFree(RFSPort->port);
                if (bytesFree < ret) {
                    ret = bytesFree;
                }
            }

            return ret;
        }

    }  // namespace RFS
}  // namespace RMFC
