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

#ifndef MSPSERIAL_H
#define MSPSERIAL_H

#include <bsp_Serial/bsp_serial.h>
#include <stdio.h>

#include "Device/bsp_Serial/DeviceSerial.h"
#include "Msp.h"

#define MSP_PORT_INBUF_SIZE 192
#ifdef USE_FLASHFS
#ifdef STM32F1
#define MSP_PORT_DATAFLASH_BUFFER_SIZE 1024
#else
#define MSP_PORT_DATAFLASH_BUFFER_SIZE 4096
#endif
#define MSP_PORT_DATAFLASH_INFO_SIZE 16
#define MSP_PORT_OUTBUF_SIZE (MSP_PORT_DATAFLASH_BUFFER_SIZE + MSP_PORT_DATAFLASH_INFO_SIZE)
#else
#define MSP_PORT_OUTBUF_SIZE 256
#endif

namespace RMFC {
    namespace MSP {

        typedef enum { MSP_PACKET_COMMAND, MSP_PACKET_REPLY } mspPacketType_e;

        typedef enum { MSP_EVALUATE_NON_MSP_DATA, MSP_SKIP_NON_MSP_DATA } mspEvaluateNonMspData_e;

        typedef int mspDescriptor_t;
        typedef struct mspPort_s {
            serialPort_t* port;  // null when port unused.
            uint32_t lastActivityMs;
            mspDescriptor_t descriptor;
            MspMessage msg;
            MspResponse res;
            // CliCmd cliCmd;
        } mspPort_t;

        void resetMspPort(mspPort_t* mspPortToReset, serialPort_t* serialPort);
        void mspSerialAllocatePorts(void);
        void mspSerialReleasePortIfAllocated(serialPort_t* serialPort);
        mspDescriptor_t getMspSerialPortDescriptor(const uint8_t portIdentifier);

        void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData);
        bool mspSerialWaiting(void);
        void mspSerialInit(void);
        int mspSerialPush(serialPortIdentifier_e portId, uint8_t cmd, uint8_t* data, int datalen);
        uint32_t mspSerialTxBytesFree(void);

    }  // namespace MSP
}  // namespace RMFC

#endif  // MSPSERIAL_H
