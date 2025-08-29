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

#include "bsp_thread.h"
#include "config/feature.h"
#include "connection_driver.h"

namespace RMFC {
    namespace RFS {

        void RefereeSerial::SerialCallbackWrapper() {
            if (callback_thread_ != nullptr) {
                callback_thread_->Set();
            }
        }

        void RefereeSerial::SerialCallbackWrapper(void* args) {
            if (args == nullptr)
                return;
            RefereeSerial* referee_serial = reinterpret_cast<RefereeSerial*>(args);
            referee_serial->SerialCallbackWrapper();
        }

        void RefereeSerial::callback_thread_func_(void* args) {
            RefereeSerial* referee_serial = reinterpret_cast<RefereeSerial*>(args);
            uint32_t len = serialRead(referee_serial->RefereePort, &referee_serial->data->data);
            if (len > 0) {
                referee_serial->data->rwstatus = Read_Status;
                referee_serial->RFSProcessor->process(
                    *referee_serial->data->data, referee_serial->data->rwstatus,
                    *referee_serial->msg, *referee_serial->res, referee_serial->RefereePort);
            }
        }

        void RefereeSerial::RefereeDataSend() {
            data->rwstatus = Write_Status;
            RFSProcessor->process(*data->data, data->rwstatus, *msg, *res, RefereePort);
        }

        bool RefereeSerial::RefereeInit() {
            // if (!featureIsEnabled(FEATURE_RFS_MSP)) {
            //     return false;
            // }

            // const serialPortConfig_t* portConfig = findSerialPortConfig(FUNCTION_RFS_MSP);
            const serialPortConfig_t* portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_HOTT);
            if (!portConfig) {
                return false;
            }

            portOptions_e options = SERIAL_NOT_INVERTED;

            if ((portConfig->identifier >= SERIAL_PORT_USART1) &&
                (portConfig->identifier <= SERIAL_PORT_USART8)) {
                options = static_cast<portOptions_e>(options | SERIAL_CHECK_TX);
            }

            // 在openSerialPort中直接设置回调函数和参数
            RefereePort =
                openSerialPort(static_cast<serialPortIdentifier_e>(portConfig->identifier),
                               FUNCTION_TELEMETRY_HOTT, nullptr, nullptr,
                               baudRates[portConfig->referee_baudrateIndex], MODE_RXTX, options);

            if (RefereePort != nullptr) {
                RFSProcessor->SetThreshold(data->threshold);

                bsp::thread_init_t thread_init = {
                    .func = callback_thread_func_,
                    .args = this,
                    .attr = callback_thread_attr_,
                };
                callback_thread_ = new bsp::EventThread(thread_init);
                SerialRegisterCallback(RefereePort, SerialCallbackWrapper, this);

                if (callback_thread_ != nullptr) {
                    return true;
                }
            }

            closeSerialPort(RefereePort);
            return false;
        }

        RefereeSerial::~RefereeSerial() {
            delete callback_thread_;
        }

    }  // namespace RFS
}  // namespace RMFC
