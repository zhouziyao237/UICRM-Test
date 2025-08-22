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
#include "PG/Referee.h"
#include "bsp_thread.h"

namespace RMFC {
    namespace RFS {
        struct RefereeData {
            uint8_t* data = nullptr;
            rwstatus_e rwstatus = Write_Status;
            serialPort_t* RefereePort = nullptr;
            RefereeProcessor RFSProcessor;
            RefereeMessage msg;
            RefereeResponse res;
        };

        bool RefereeInit(const RefereeConfig_t* RefereeConfig);
        void deleteThread_Referee();
        void RefereeDataSend();

        void Referee_CallbackWrapper(void* args);
        void Referee_thread_func_(void* args);
        extern RefereeData* referee_data;
        extern bsp::EventThread* callback_thread;
        extern driver::ConnectionDriver* connection_driver;
        const osThreadAttr_t Referee_thread_attr = {.name = "RefereeUpdateTask",
                                                     .attr_bits = osThreadDetached,
                                                     .cb_mem = nullptr,
                                                     .cb_size = 0,
                                                     .stack_mem = nullptr,
                                                     .stack_size = 256 * 2,
                                                     .priority = (osPriority_t)osPriorityHigh,
                                                     .tz_module = 0,
                                                     .reserved = 0};

    }  // namespace RFS
}  // namespace RMFC
