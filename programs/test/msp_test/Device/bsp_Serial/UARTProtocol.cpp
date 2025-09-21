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
// Created by Administrator on 2025/9/21.
//

#include "UARTProtocol.h"

namespace RMFC {

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
}  // namespace RMFC