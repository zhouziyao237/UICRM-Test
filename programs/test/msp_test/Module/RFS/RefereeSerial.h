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

#pragma once

#include "Device/bsp_Serial/DeviceSerial.h"
#include "Module/RFS/RefereeProcessor.h"
#include "config/feature.h"
#include "platform.h"

namespace RMFC {

    // 前向声明
    class UserInterface;

    // todo：创建一个结构体来接收分配裁判系统的数据
    typedef int refereeDescriptor_t;
    typedef struct RefereePort_s {
        serialPort_t* port;  // null when port unused.
        timeUs_t lastActivityMs;
        refereeDescriptor_t descriptor;
        RFS::RefereeProtocol* Referee_protocol;
        UserInterface* UI;

        uint8_t data;
        RFS::package_t package;
    } RefereePort_t;

// ---------------------------------------------- 分割线 --------------------------------------------


    namespace RFS {

        typedef struct {
            features_e features;
            serialPortFunction_e serialPortFunction;
            baudrateFunctionIndex_e baudRatesIndex;
        } RefereeProtConfig_t;

        // todo: 在这里创建一个结构提来记录串口的给裁判系统ID和在裁判系统中细分的功能
        typedef enum {
            RFS_MSP_NONE = 0,
            RFS_MSP_REFEREE = (1 << 0),
            RFS_MSP_OSD_UI = (1 << 1),
            RFS_MSP_RX = (1 << 2),
        } RefereePortFunction_e;

        extern RefereePort_t RefereePort[MAX_RFS_PORT_COUNT];

        serialPort_t* FindRefereeProtocol(RefereeProtConfig_t ProtConfig);

        void resetRefereePort(RefereePort_t* RFSPortToReset, serialPort_t* serialPort);
        void RefereeSerialReleasePortIfAllocated(serialPort_t* serialPort);
        void RefereeSerialAllocatePorts(void);
        refereeDescriptor_t getRefereeSerialPortDescriptor(const uint8_t portIdentifier);

        void RefereeSerialProcess();
        bool RefereeSerialWaiting(void);
        void RefereeSerialInit(void);
        int RefereeSerialPush(serialPortIdentifier_e portId, uint8_t cmd, uint8_t* data, int datalen);
        uint32_t RefereeSerialTxBytesFree(void);


    }  // namespace RFS
}  // namespace RMFC
