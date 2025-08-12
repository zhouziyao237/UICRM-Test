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

#include "DeviceSerial.h"

#include "bsp_uart.h"
#include "bsp_usb.h"

namespace RMFC {

    const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT] = {
    #ifdef SerialUART_VCP
        SERIAL_PORT_USB_VCP,
    #endif
    #ifdef SerialUART_1
        SERIAL_PORT_USART1,
    #endif
    #ifdef SerialUART_2
        SERIAL_PORT_USART2,
    #endif
    #ifdef SerialUART_3
        SERIAL_PORT_USART3,
    #endif
    #ifdef SerialUART_4
        SERIAL_PORT_UART4,
    #endif
    #ifdef SerialUART_5
        SERIAL_PORT_UART5,
    #endif
    #ifdef SerialUART_6
        SERIAL_PORT_USART6,
    #endif
    #ifdef SerialUART_7
        SERIAL_PORT_USART7,
    #endif
    #ifdef SerialUART_8
        SERIAL_PORT_USART8,
    #endif
    };

    static uint8_t serialPortCount;

    static serialPortUsage_t serialPortUsageList[SERIAL_PORT_COUNT];

    const uint32_t baudRates[] = {0,       9600,    19200,   38400,  57600,  115200,
                                  230400,  250000,  400000,  460800, 500000, 921600,
                                  1000000, 1500000, 2000000, 2470000};  // see baudRate_e

    #define BAUD_RATE_COUNT (sizeof(baudRates) / sizeof(baudRates[0]))

    baudRate_e lookupBaudRateIndex(uint32_t baudRate)
    {
        for (uint8_t index = 0; index < BAUD_RATE_COUNT; index++) {
            if (baudRates[index] == baudRate) {
                return static_cast<baudRate_e>(index);
            }
        }
        return BAUD_AUTO;
    }

    serialPortUsage_t *findSerialPortUsageByIdentifier(const serialPortIdentifier_e identifier) {
        uint8_t index;
        for (index = 0; index < SERIAL_PORT_COUNT; index++) {
            serialPortUsage_t *candidate = &serialPortUsageList[index];
            if (candidate->identifier == identifier) {
                return candidate;
            }
        }
        return NULL;
    }

    serialPortUsage_t *findSerialPortUsageByPort(const serialPort_t *serialPort) {
        uint8_t index;
        for (index = 0; index < SERIAL_PORT_COUNT; index++) {
            serialPortUsage_t *candidate = &serialPortUsageList[index];
            if (candidate->serialPort == serialPort) {
                return candidate;
            }
        }
        return NULL;
    }

    serialPortUsage_t *findSerialPortUsageByFunction(const serialPortFunction_e function) {
        uint8_t index;
        for (index = 0; index < SERIAL_PORT_COUNT; index++) {
            serialPortUsage_t *candidate = &serialPortUsageList[index];
            if (candidate->function == function) {
                return candidate;
            }
        }
        return NULL;
    }



    void serialInit(serialPortIdentifier_e serialPortToDisable)
    {
        serialPortCount = SERIAL_PORT_COUNT;
        memset(&serialPortUsageList, 0, sizeof(serialPortUsageList));

        for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
            serialPortUsageList[index].identifier = serialPortIdentifiers[index];

            if (serialPortToDisable != SERIAL_PORT_NONE) {
                if (serialPortUsageList[index].identifier == serialPortToDisable) {
                    serialPortUsageList[index].identifier = SERIAL_PORT_NONE;
                    serialPortCount--;
                }
            }
        }
    }

}  // namespace RMFC
