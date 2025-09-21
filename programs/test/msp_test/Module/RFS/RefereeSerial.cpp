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

namespace RMFC {
    namespace RFS {

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

    }  // namespace RFS
}  // namespace RMFC
