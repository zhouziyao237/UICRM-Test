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
// Created by Administrator on 2025/8/19.
//

#pragma once

#include "Device/bsp_Serial/DeviceSerial.h"
#include "Device/bsp_Serial/UARTProtocol.h"
#include "MiniPCHost.h"
#include "MiniPCHostParser.h"
#include "MiniPCHostProcessor.h"
#include "connection_driver.h"

extern "C" {
#include "config/MiniPC_protocol.h"
#include "config/MiniPC_protocol_data.h"
}

namespace RMFC {
    namespace MiniPC {

        typedef enum {
            MiniPC_RESULT_ACK = 1,
            MiniPC_RESULT_ERROR = -1,
            MiniPC_RESULT_NO_REPLY = 0,
            MiniPC_RESULT_CMD_UNKNOWN = -2,  // don't know how to process command, try next handler
        } MiniPCResult_e;

        typedef enum { Read_Status = 0, Write_Status = 1 } rwstatus_e;

        typedef struct {
            MiniPCMessage msg;
            MiniPCResponse res;
            rwstatus_e rwstatus;
            int length;
        } package_t;

        class MiniPCProcessor : public driver::ConnectionDriver {
          public:
            MiniPCProcessor();

            bool ProcessReceive(char data, package_t& package);

            bool ProcessTransmit(package_t& package, serialPort_t* SerialMiniPC);

            MiniPCResult_e ReceiveCommand(MiniPCMessage& m);

            MiniPCResult_e SendCommand(MiniPCMessage& m, MiniPCResponse& r);

            void sendResponseV1(serialPort_t* SerialMiniPC, MiniPCResponse& r);

            MiniPCdata* MiniPCdata_;

          private:
            MiniPCParser _MiniPCParser;
        };

        class MiniPCProtocol : public MiniPCProcessor, UARTProtocol {
          public:
            explicit MiniPCProtocol(serialPort_t* UARTProt_, bool startThread = true);
            ~MiniPCProtocol();

            void SetThread();

            void setUARTPort(serialPort_t* SetUARTProt);
            serialPort_t* getUARTPort() const;
            bool isValid() const;

            void Receive(void* args) override;
            int Transmit(int cmd_id) override;

        private:
            uint8_t BufferRxdata;
            package_t BufferPackage;
        };

    }  // namespace MiniPC
}  // namespace RMFC