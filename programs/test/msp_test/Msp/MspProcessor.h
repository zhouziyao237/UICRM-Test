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

#ifndef _ESPFC_MSP_PROCESSOR_H_
#define _ESPFC_MSP_PROCESSOR_H_

#include "Msp.h"
#include "MspParser.h"
#include "MspProcessor.h"
#include "Device/serial/bsp_serial.h"
// #include "Model/Model.h"
// #include "Hardware.h"
// #include "platform.h"

namespace RMFC {
    namespace Msp {
        class MspProcessor {
          public:
            // MspProcessor(Model& model): _model(model) {}
            void InitMSP(serialPort_t* MSP, serialPort_t* Debug);

            bool process(char c, MspMessage& msg, MspResponse& res);

            void processCommand(MspMessage& m, MspResponse& r);

            void sendResponse(MspResponse& r);

            void sendResponseV1(MspResponse& r);

            void sendResponseV2(MspResponse& r);

            void postCommand();

            bool debugSkip(uint8_t cmd);

            void debugMessage(const MspMessage& m);

            void debugResponse(const MspResponse& r);

          private:
            // Model& _model;
            serialPort_t* SerialDebug;
            serialPort_t* SerialMSP;
            MspParser _parser;
            bool _reboot;
        };
    }  // namespace Msp
}  // namespace Espfc

#endif
