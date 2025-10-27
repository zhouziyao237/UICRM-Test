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

#pragma once

#include "Device/bsp_Serial/DeviceSerial.h"
#include "Module/MiniPC/MiniPCHostProcessor.h"
#include "config/feature.h"
#include "platform.h"

namespace RMFC {
    namespace MiniPC {

        typedef int MiniPCDescriptor_t;
        typedef struct MiniPCPort_s {
            serialPort_t* port;  // null when port unused.
            timeUs_t lastActivityMs;
            MiniPCDescriptor_t descriptor;
            MiniPCProtocol* MiniPC_protocol;

            uint8_t data;
            package_t package;
        } MiniPCPort_t;

        typedef struct {
            features_e features;
            serialPortFunction_e serialPortFunction;
            baudrateFunctionIndex_e baudRatesIndex;
        } MiniPCProtConfig_t;

        // todo: 在这里创建一个结构提来记录串口的给裁判系统ID和在裁判系统中细分的功能
        typedef enum {
            MiniPC_MSP_NONE = 0,
            MiniPC_MSP_REFEREE = (1 << 0),
            MiniPC_MSP_OSD_UI = (1 << 1),
            MiniPC_MSP_RX = (1 << 2),
        } RefereePortFunction_e;

        extern MiniPCPort_t MiniPCPort[MAX_RFS_PORT_COUNT];

        serialPort_t* FindRefereeProtocol(MiniPCProtConfig_t ProtConfig);

        void resetRefereePort(MiniPCPort_t* RFSPortToReset, MiniPCPort_t* serialPort);
        void MiniPCSerialReleasePortIfAllocated(serialPort_t* serialPort);
        void MiniPCSerialAllocatePorts(void);
        MiniPCDescriptor_t getMiniPCSerialPortDescriptor(const uint8_t portIdentifier);

        void MiniPCSerialProcess();
        bool MiniPCSerialWaiting(void);
        void MiniPCSerialInit(void);
        int MiniPCSerialPush(serialPortIdentifier_e portId, uint8_t cmd, uint8_t* data,
                              int datalen);
        uint32_t MiniPCSerialTxBytesFree(void);
    }  // namespace MiniPC
}  // namespace RMFC
