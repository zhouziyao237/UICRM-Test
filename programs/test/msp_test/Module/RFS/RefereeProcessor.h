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
#include "Device/bsp_Serial/bsp_Serial.h"
#include "Referee.h"
#include "RefereeParser.h"
#include "RefereeProcessor.h"

extern "C" {
#include "config/Referee_protocol.h"
#include "config/Referee_protocol_data.h"
}

namespace RMFC {
    namespace RFS {
        typedef enum {
            MSP_RESULT_ACK = 1,
            MSP_RESULT_ERROR = -1,
            MSP_RESULT_NO_REPLY = 0,
            MSP_RESULT_CMD_UNKNOWN = -2,  // don't know how to process command, try next handler
        } RefereeResult_e;

        typedef enum { Read_Status = 0, Write_Status = 1 } rwstatus_e;

        class RefereeProcessor {
          public:
            bool process(char data, rwstatus_e rwstatus, RefereeMessage& msg, RefereeResponse& res,
                         serialPort_t* SerialRFS);

            RefereeResult_e ReceiveCommand(RefereeMessage& m);

            RefereeResult_e SendCommand(RefereeMessage& m, RefereeResponse& r);

            void sendResponseV1(serialPort_t* SerialRFS, RefereeResponse& r);

          private:
            RefereeParser _RefereeParser;
            Referee _Referee;
        };
    }  // namespace RFS
}  // namespace RMFC