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

#include "RefereeSerial.h"
#include "PG/Referee.h"
#include "bsp_thread.h"
#include "connection_driver.h"

namespace RMFC {
    namespace RFS {

        RefereeData* referee_data = nullptr;
        bsp::EventThread* callback_thread = nullptr;
        driver::ConnectionDriver* connection_driver = nullptr;

        bool RefereeInit(const RefereeConfig_t* RefereeConfig) {

            const serialPortConfig_t* portConfig = findSerialPortConfig(FUNCTION_RFS_MSP);
            if (!portConfig) {
                return false;
            }

            serialPort_t* RefereePort =
                openSerialPort(static_cast<serialPortIdentifier_e>(portConfig->identifier),
                FUNCTION_RFS_MSP,
                Referee_CallbackWrapper,
                nullptr,
                RefereeConfig->BaudRate,
                RefereeConfig->portShared ? MODE_RXTX : MODE_RX,
                SERIAL_INVERTED
                );


            if (RefereePort != nullptr) {
                referee_data->RFSProcessor.SetThreshold(RefereeConfig->threshold);

                bsp::thread_init_t Referee_thread_init = {
                    .func = Referee_thread_func_,
                    .args = nullptr,
                    .attr = Referee_thread_attr,
                };
                callback_thread = new bsp::EventThread(Referee_thread_init);
                return true;
            } else {
                return false;
            }
        }

        void deleteThread_Referee() {
            delete callback_thread;
        }

        void Referee_CallbackWrapper(void* args) {
            UNUSED(args);
            callback_thread->Set();
        }

        void Referee_thread_func_(void* args) {
            UNUSED(args);
            uint32_t len = serialRead(referee_data->RefereePort, &referee_data->data);
            if (len > 0) {
                referee_data->rwstatus = Read_Status;
                referee_data->RFSProcessor.process(*referee_data->data, referee_data->rwstatus, referee_data->msg,
                                              referee_data->res, referee_data->RefereePort);
            }
            referee_data->data = nullptr;
        }

        void RefereeDataSend() {
            referee_data->rwstatus = Write_Status;
            referee_data->RFSProcessor.process(*referee_data->data, referee_data->rwstatus, referee_data->msg,
                                          referee_data->res, referee_data->RefereePort);
        }

    }  // namespace RFS
}  // namespace RMFC
