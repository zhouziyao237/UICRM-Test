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

#include <RefereeProcessor.h>

#include "Device/bsp_Serial/DeviceSerial.h"
#include "Device/bsp_Serial/bsp_Serial.h"
#include "bsp_error_handler.h"
#include "bsp_thread.h"
#include "config/feature.h"

namespace RMFC {
    namespace RFS {

        typedef struct {
            features_e features;
            serialPortFunction_e serialPortFunction;
            baudrateFunctionIndex_e baudRatesIndex;
        } RefereeProtConfig_t;

        class RefereeProtocol : public RefereeProcessor {
          public:
            explicit RefereeProtocol();
            ~RefereeProtocol();
            bool RefereeInit(RefereeProtConfig_t ProtConfig);
            void SetThreshold(uint32_t threshold);
            void InitTask();
            void Transmit(int cmd_id);

          protected:
            serialPort_t* UARTProt_;

          private:
            uint8_t* data;
            package_t* package;
            static void CallbackWrapper(void* args);
            bsp::EventThread* callback_thread_ = nullptr;
            static void callback_thread_func_(void* args);
            const osThreadAttr_t callback_thread_attr_ = {.name = "RefereeProtocolUpdateTask",
                                                          .attr_bits = osThreadDetached,
                                                          .cb_mem = nullptr,
                                                          .cb_size = 0,
                                                          .stack_mem = nullptr,
                                                          .stack_size = 256 * 4,
                                                          .priority = (osPriority_t)osPriorityHigh,
                                                          .tz_module = 0,
                                                          .reserved = 0};
        };

    }  // namespace RFS
}  // namespace RMFC
