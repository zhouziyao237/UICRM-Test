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
// Created by Administrator on 2025/8/10.
//

#include "Model.h"

namespace RMFC {
    Device::SerialDevice* getSerialStream(SerialPort i) {
        return state.serial[i].stream;
    }

    Device::SerialDevice* getSerialStream(SerialFunction sf) {
        for (size_t i = 0; i < SERIAL_UART_COUNT; i++) {
            if (config.serial[i].functionMask & sf)
                return state.serial[i].stream;
        }
        return nullptr;
    }

    int getSerialIndex(SerialPortId id) {
        switch (id) {
        #ifdef ESPFC_SERIAL_0
            case SERIAL_ID_UART_1:
                return SERIAL_UART_0;
        #endif
        #ifdef ESPFC_SERIAL_1
            case SERIAL_ID_UART_2:
                return SERIAL_UART_1;
        #endif
        #ifdef ESPFC_SERIAL_2
            case SERIAL_ID_UART_3:
                return SERIAL_UART_2;
        #endif
        #ifdef ESPFC_SERIAL_USB
            case SERIAL_ID_USB_VCP:
                return SERIAL_USB;
        #endif
        #ifdef ESPFC_SERIAL_SOFT_0
            case SERIAL_ID_SOFTSERIAL_1:
                return SERIAL_SOFT_0;
        #endif
            default:
                break;
        }
        return -1;
    }

}  // namespace RMFC
