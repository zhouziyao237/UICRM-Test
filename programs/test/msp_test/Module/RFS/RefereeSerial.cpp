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

        UARTProtocol::UARTProtocol(serialPort_t* uart) : UARTProt_(uart) {
            SerialRegisterCallback(UARTProt_, CallbackWrapper, this);
            bsp::thread_init_t thread_init = {
                .func = callback_thread_func_,
                .args = this,
                .attr = callback_thread_attr_,
            };
            callback_thread_ = new bsp::EventThread(thread_init);
        }

        UARTProtocol::~UARTProtocol() {
            delete callback_thread_;
        }

        void UARTProtocol::CallbackWrapper(void* args) {
            UARTProtocol* uart_protocol_ = reinterpret_cast<UARTProtocol*>(args);
            uart_protocol_->callback_thread_->Set();
        }

        void UARTProtocol::callback_thread_func_(void* args) {
            UARTProtocol* uart_protocol_ = reinterpret_cast<UARTProtocol*>(args);
            uart_protocol_->Receive(nullptr);
        }

        void UARTProtocol::Receive(void* args) {
            UNUSED(args);
        }

        int UARTProtocol::Transmit(int cmd_id) {
            UNUSED(cmd_id);
            return -1;
        }

        // void UARTProtocol::callback_thread_func_(void* args) {
        //     UARTProtocol* uart_protocol_ = reinterpret_cast<UARTProtocol*>(args);
        //     if (uart_protocol_->Interface_ && uart_protocol_->Interface_->callback_thread_func) {
        //         uart_protocol_->Interface_->callback_thread_func(args);
        //     }
        // }
        //
        // int UARTProtocol::Transmit(int cmd_id) {
        //     if (Interface_ && Interface_->Transmit) {
        //         return Interface_->Transmit(cmd_id);
        //     }
        //     return -1;
        // }

        serialPort_t* FindRefereeProtocol(RefereeProtConfig_t ProtConfig) {
            // todo: 没写FEATURE_RFS_MSP的设置，写完再开
            // if (!featureIsEnabled(ProtConfig.features)) {
            //     return false;
            // }

            const serialPortConfig_t* port = findSerialPortConfig(ProtConfig.serialPortFunction);
            if (port == nullptr) {
                return nullptr;
            }

            portOptions_e options = SERIAL_NOT_INVERTED;

            if ((port->identifier >= SERIAL_PORT_USART1) &&
                (port->identifier <= SERIAL_PORT_USART8)) {
                options = static_cast<portOptions_e>(options | SERIAL_CHECK_TX);
            }

            serialPort_t* Prot = openSerialPort(
                static_cast<serialPortIdentifier_e>(port->identifier),
                ProtConfig.serialPortFunction, NULL, NULL,
                baudRates[getBaudrateIndexByFunction(port, ProtConfig.baudRatesIndex)], MODE_RXTX,
                options);

            // InitTask();

            return Prot;
        }

        RefereeProtocol::RefereeProtocol(serialPort_t* Prot) : UARTProtocol(Prot) {
            // 设置100ms离线阈值
            SetThreshold(100);
        }

        RefereeProtocol::~RefereeProtocol() {
        }

        void RefereeProtocol::Receive(void* args) {
            UNUSED(args);
            package.rwstatus = Read_Status;
            uint32_t len = serialRead(UARTProt_, reinterpret_cast<uint8_t**>(data));
            if (len > 0) {
                ProcessReceive(data, package);
            }
        }

        int RefereeProtocol::Transmit(int cmd_id) {
            package.rwstatus = Write_Status;
            package.res->cmd = cmd_id;
            ProcessTransmit(package, UARTProt_);
            return package.length;
        }

    }  // namespace RFS
}  // namespace RMFC
