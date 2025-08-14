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
// Created by Administrator on 2025/8/9.
//

#ifndef _MSP_PROCESSOR_H_
#define _MSP_PROCESSOR_H_

#include "Device/bsp_serial/bsp_serial.h"
#include "Msp.h"
#include "MspParser.h"
#include "MspProcessor.h"

// #include "Model/Model.h"
// #include "Hardware.h"
// #include "platform.h"

namespace RMFC {
    namespace MSP {

        typedef enum {
            MSP_RESULT_ACK = 1,
            MSP_RESULT_ERROR = -1,
            MSP_RESULT_NO_REPLY = 0,
            MSP_RESULT_CMD_UNKNOWN = -2,  // don't know how to process command, try next handler
        } mspResult_e;

        class MspProcessor {
          public:
            // MspProcessor(Model& model): _model(model) {}
            void InitMSP(serialPort_t* Debug);

            bool process(char data, MspMessage& msg, MspResponse& res, serialPort_t* SerialMSP);
            // msp post process function, used for gracefully handling reboots, etc.

            typedef void (*mspPostProcessFnPtr)(struct serialPort_s* port);

            mspResult_e processCommand(MspMessage& m, MspResponse& r,
                                       mspPostProcessFnPtr* mspPostProcessFn);

            void sendResponse(serialPort_t* SerialMSP, MspResponse& r);

            void sendResponseV1(serialPort_t* SerialMSP, MspResponse& r);

            void sendResponseV2(serialPort_t* SerialMSP, MspResponse& r);

            void postCommand();

            bool debugSkip(uint8_t cmd);

            void debugMessage(const MspMessage& m);

            void debugResponse(const MspResponse& r);

          private:
            // Model& _model;
            serialPort_t* SerialDebug;
            MspParser _parser;
        };
    }  // namespace MSP
}  // namespace RMFC

#endif
