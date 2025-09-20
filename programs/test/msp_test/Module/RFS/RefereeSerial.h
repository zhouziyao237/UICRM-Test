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
#include "bsp_error_handler.h"
#include "bsp_thread.h"
#include "config/feature.h"

namespace RMFC {
    namespace RFS {


        class UARTProtocol : public RefereeProcessor {
          public:
            explicit UARTProtocol(serialPort_t* uart);
            ~UARTProtocol();

            virtual int Transmit(int cmd_id);
            virtual void Receive(void* args);

          protected:
            serialPort_t* UARTProt_;

          private:

            static void CallbackWrapper(void* args);
            static void callback_thread_func_(void* args);
            bsp::EventThread* callback_thread_ = nullptr;
            const osThreadAttr_t callback_thread_attr_ = {.name = "RefereeTask",
                                                          .attr_bits = osThreadDetached,
                                                          .cb_mem = nullptr,
                                                          .cb_size = 0,
                                                          .stack_mem = nullptr,
                                                          .stack_size = 256 * 4,
                                                          .priority = (osPriority_t)osPriorityHigh,
                                                          .tz_module = 0,
                                                          .reserved = 0};
        };

        typedef struct {
            features_e features;
            serialPortFunction_e serialPortFunction;
            baudrateFunctionIndex_e baudRatesIndex;
        } RefereeProtConfig_t;

        serialPort_t* FindRefereeProtocol(RefereeProtConfig_t ProtConfig);

        class RefereeProtocol : public UARTProtocol{
        public:
            explicit RefereeProtocol(serialPort_t* Prot);
            ~RefereeProtocol();

            void Receive(void* args);
            int Transmit(int cmd_id);

        protected:
            uint8_t data;
            package_t package;
        };

    }  // namespace RFS
}  // namespace RMFC
