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

#include "bsp_error_handler.h"
#include "bsp_thread.h"

namespace RMFC {
    namespace RFS {

        struct data_s {
            uint32_t threshold = 100;
            uint8_t* data = nullptr;
            rwstatus_e rwstatus = Write_Status;
        };

        class RefereeSerial : public driver::ConnectionDriver {
          public:
            ~RefereeSerial();
            bool RefereeInit();
            void RefereeDataSend();

          private:
            void SerialCallbackWrapper();
            static void SerialCallbackWrapper(void* args);
            serialPort_t* RefereePort;
            data_s* data;
            RefereeProcessor* RFSProcessor;
            RefereeMessage* msg;
            RefereeResponse* res;
            bsp::EventThread* callback_thread_ = nullptr;
            const osThreadAttr_t callback_thread_attr_ = {
                .name = "RefereeUpdateTask",
                .attr_bits = osThreadDetached,
                .cb_mem = nullptr,
                .cb_size = 0,
                .stack_mem = nullptr,
                .stack_size = 1024,
                .priority = (osPriority_t)osPriorityNormal,
                .tz_module = 0,
                .reserved = 0};

            static void callback_thread_func_(void* args);
        };

    }  // namespace RFS
}  // namespace RMFC
