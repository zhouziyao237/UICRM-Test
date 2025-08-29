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

        RefereeProtocol::RefereeProtocol() : RefereeProcessor() {
        }

        RefereeProtocol::~RefereeProtocol() {
            delete callback_thread_;
        }

        bool RefereeProtocol::RefereeInit(RefereeProtConfig_t ProtConfig) {
            // todo: 没写FEATURE_RFS_MSP的设置，写完再开
            // if (!featureIsEnabled(ProtConfig.features)) {
            //     return false;
            // }

            const serialPortConfig_t* port = findSerialPortConfig(ProtConfig.serialPortFunction);
            if (port == nullptr) {
                return false;
            }

            portOptions_e options = SERIAL_NOT_INVERTED;

            if ((port->identifier >= SERIAL_PORT_USART1) &&
                (port->identifier <= SERIAL_PORT_USART8)) {
                options = static_cast<portOptions_e>(options | SERIAL_CHECK_TX);
            }

            UARTProt_ = openSerialPort(
                static_cast<serialPortIdentifier_e>(port->identifier),
                ProtConfig.serialPortFunction, NULL, NULL,
                baudRates[getBaudrateIndexByFunction(port, ProtConfig.baudRatesIndex)], MODE_RXTX,
                options);

            InitTask();

            return true;
        }

        void RefereeProtocol::SetThreshold(uint32_t threshold) {
            SetThreshold(threshold);
        }

        void RefereeProtocol::InitTask() {
            SerialRegisterCallback(UARTProt_, CallbackWrapper, this);
            bsp::thread_init_t thread_init = {
                .func = callback_thread_func_,
                .args = this,
                .attr = callback_thread_attr_,
            };
            callback_thread_ = new bsp::EventThread(thread_init);
        }

        void RefereeProtocol::CallbackWrapper(void* args) {
            RefereeProtocol* protocol_ = reinterpret_cast<RefereeProtocol*>(args);
            protocol_->callback_thread_->Set();
        }

        void RefereeProtocol::callback_thread_func_(void* args) {
            RefereeProtocol* protocol_ = reinterpret_cast<RefereeProtocol*>(args);
            protocol_->package->rwstatus = Read_Status;
            uint32_t len = serialRead(protocol_->UARTProt_, &protocol_->data);
            if (len > 0) {
                protocol_->ProcessReceive(*protocol_->data, *protocol_->package);
            }
        }

        void RefereeProtocol::Transmit(int cmd_id) {
            package->rwstatus = Write_Status;
            package->res->cmd = cmd_id;
            ProcessTransmit(*package, UARTProt_);
        }
    }  // namespace RFS
}  // namespace RMFC
