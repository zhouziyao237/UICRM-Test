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
// Created by Administrator on 2025/10/26.
//

#include "MiniPCSerial.h"

#include "PG/minipc.h"
#include "bsp_os.h"
#include "cmsis_os.h"
#include "target.h"

namespace RMFC {
    namespace MiniPC {
        MiniPCPort_t MiniPCPort[MAX_RFS_PORT_COUNT];

        void resetMiniPCPort(MiniPCPort_t* MiniPCPortToReset, serialPort_t* serialPort) {
            *MiniPCPortToReset = MiniPCPort_t{};
            MiniPCPortToReset->MiniPC_protocol->setUARTPort(serialPort);
        }

        void MiniPCSerialReleasePortIfAllocated(serialPort_t* serialPort) {
            for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                MiniPCPort_t* candidateRefereePort = &MiniPCPort[portIndex];
                if (candidateRefereePort->MiniPC_protocol->getUARTPort() == serialPort) {
                    closeSerialPort(serialPort);
                    *candidateRefereePort = MiniPCPort_t{};
                }
            }
        }

        MiniPCDescriptor_t getMiniPCSerialPortDescriptor(const uint8_t portIdentifier) {
            for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                MiniPCPort_t* candidateMiniPCPort = &MiniPCPort[portIndex];
                serialPort_t* port = candidateMiniPCPort->getPort();  // 使用便捷方法
                if (port && port->identifier == portIdentifier) {
                    return candidateMiniPCPort->descriptor;
                }
            }
            return -1;
        }

        /*
         * Process MiniPC commands from serial ports configured as MiniPC ports.
         *
         * Called periodically by the scheduler.
         */
        void MiniPCSerialProcess() {
            for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                MiniPCPort_t* const MiniPCProt = &MiniPCPort[portIndex];
                if (!MiniPCProt->MiniPC_protocol->getUARTPort()) {
                    continue;
                }

                // if (!MiniPCProt->EnableFunction) {
                //     continue;
                // }

                // There are bytes incoming - abort pending request
                MiniPCProt->lastActivityMs = bsp::GetHighresTickMilliSec();

                MiniPCProt->SerialRxpackage.rwstatus = Read_Status;

                uint8_t* SerialRxdata = nullptr;
                uint32_t len = serialRead(MiniPCProt->MiniPC_protocol->getUARTPort(), &SerialRxdata);
                if (len > 0) {
                    char* str = reinterpret_cast<char*>(SerialRxdata);
                    for (uint32_t i = 0; i < len; i++) {
                        MiniPCProt->MiniPC_protocol->ProcessReceive(*str, MiniPCProt->SerialRxpackage);
                        str++;
                    }
                }
            }
        }

        void MiniPCSerialAllocatePorts(void) {
            uint8_t portIndex = 0;
            const serialPortConfig_t* portConfig = findSerialPortConfig(FUNCTION_MiniPCHost);
            while (portConfig && portIndex < MAX_MSP_PORT_COUNT) {
                MiniPCPort_t* minipcPort = &MiniPCPort[portIndex];

                // 检查当前端口是否已被占用
                if (minipcPort->getPort()) {  // 使用新添加的方法
                    portIndex++;
                    continue;
                }

                portOptions_e options = SERIAL_NOT_INVERTED;

                if (MiniPCConfig()->halfDuplex) {
                    options = static_cast<portOptions_e>(options | SERIAL_CHECK_TX);
                } else if ((portConfig->identifier >= SERIAL_PORT_USART1) &&
                           (portConfig->identifier <= SERIAL_PORT_USART8)) {
                    options = static_cast<portOptions_e>(options | SERIAL_CHECK_TX);
                }

                serialPort_t* serialPort = openSerialPort(
                    static_cast<serialPortIdentifier_e>(portConfig->identifier),
                    FUNCTION_MiniPCHost,
                    NULL,
                    NULL,
                    baudRates[portConfig->referee_baudrateIndex],
                    MODE_RXTX,
                    options);

                if (serialPort) {
                    // 创建临时协议对象
                    MiniPCProtocol* tempProtocol = new MiniPCProtocol(serialPort, false);

                    // 检查配置是否正确
                    if (tempProtocol && tempProtocol->isValid()) {  // 假设有这样的验证方法
                        // 释放原有的协议对象（如果存在）
                        if (minipcPort->MiniPC_protocol) {
                            delete minipcPort->MiniPC_protocol;
                        }

                        // 赋值新创建的对象
                        minipcPort->MiniPC_protocol = tempProtocol;
                        portIndex++;  // 只有成功创建后才增加索引
                    } else {
                        // 如果配置不正确，删除临时对象
                        delete tempProtocol;
                        closeSerialPort(serialPort);  // 关闭已打开的串口
                    }
                }

                portConfig = findNextSerialPortConfig(FUNCTION_MiniPCHost);
            }
        }

        bool MiniPCSerialWaiting(void) {
            for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                MiniPCPort_t* const MiniPCProt = &MiniPCPort[portIndex];
                if (!MiniPCProt->MiniPC_protocol->getUARTPort()) {
                    continue;
                }

                if (serialRxBytesWaiting(MiniPCProt->MiniPC_protocol->getUARTPort())) {
                    return true;
                }
            }
            return false;
        }

        void MiniPCSerialInit(void) {
            // todo: 没写FEATURE_REF_PC_MSP的设置，写完再开
            // if (!featureIsEnabled(FEATURE_REF_PC_MSP)) {
            //     return false;
            // }
            *MiniPCPort = MiniPCPort_t{};
            MiniPCSerialAllocatePorts();
            print("[INFO] MiniPC Init!");
        }

        int MiniPCSerialPush(serialPortIdentifier_e portId, uint8_t cmd, uint8_t* data, int datalen) {
            UNUSED(cmd);
            int ret = 0;

            for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                MiniPCPort_t* const MiniPCProt = &MiniPCPort[portIndex];

                // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
                if (!MiniPCProt->MiniPC_protocol->getUARTPort()
#ifndef USE_MSP_PUSH_OVER_VCP
                    || MiniPCProt->MiniPC_protocol->getUARTPort()->identifier == SERIAL_PORT_USB_VCP
#endif
                    || (portId != SERIAL_PORT_ALL && MiniPCProt->MiniPC_protocol->getUARTPort()->identifier != portId)) {
                    continue;
                }

                // 直接发送数据
                serialBeginWrite(MiniPCProt->MiniPC_protocol->getUARTPort());
                serialWriteBuf(MiniPCProt->MiniPC_protocol->getUARTPort(), data, datalen);
                serialEndWrite(MiniPCProt->MiniPC_protocol->getUARTPort());

                ret = datalen;
            }
            return ret;
        }

        uint32_t MiniPCSerialTxBytesFree(void) {
            uint32_t ret = UINT32_MAX;

            for (int portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
                MiniPCPort_t* const MiniPCProt = &MiniPCPort[portIndex];
                if (!MiniPCProt->MiniPC_protocol->getUARTPort()) {
                    continue;
                }

                // XXX Kludge!!! Avoid zombie VCP port (avoid VCP entirely for now)
                if (MiniPCProt->MiniPC_protocol->getUARTPort()->identifier == SERIAL_PORT_USB_VCP) {
                    continue;
                }

                const uint32_t bytesFree = serialTxBytesFree(MiniPCProt->MiniPC_protocol->getUARTPort());
                if (bytesFree < ret) {
                    ret = bytesFree;
                }
            }

            return ret;
        }
    }
}  // namespace RMFC