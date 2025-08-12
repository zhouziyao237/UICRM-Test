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

#ifndef DEVICESERIAL_H
#define DEVICESERIAL_H

#include "Target.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "bsp_serial.h"

#define SERIAL_PORT_IDENTIFIER_TO_INDEX(x) \
    (((x) <= SERIAL_PORT_USART8) ? (x) : (RESOURCE_SOFT_OFFSET + ((x) - SERIAL_PORT_SOFTSERIAL1)))
#define SERIAL_PORT_IDENTIFIER_TO_UARTDEV(x) ((x) - SERIAL_PORT_USART1 + UARTDEV_1)

namespace RMFC {
    typedef enum {
        PORTSHARING_UNUSED = 0,
        PORTSHARING_NOT_SHARED,
        PORTSHARING_SHARED
    } portSharing_e;

    typedef enum {
        FUNCTION_NONE = 0,
        FUNCTION_MSP = (1 << 0),                  // 1
        FUNCTION_GPS = (1 << 1),                  // 2
        FUNCTION_TELEMETRY_FRSKY_HUB = (1 << 2),  // 4
        FUNCTION_TELEMETRY_HOTT = (1 << 3),       // 8
        FUNCTION_TELEMETRY_LTM = (1 << 4),        // 16
        FUNCTION_TELEMETRY_SMARTPORT = (1 << 5),  // 32
        FUNCTION_RX_SERIAL = (1 << 6),            // 64
        FUNCTION_BLACKBOX = (1 << 7),             // 128
        FUNCTION_TELEMETRY_MAVLINK = (1 << 9),    // 512
        FUNCTION_ESC_SENSOR = (1 << 10),          // 1024
        FUNCTION_VTX_SMARTAUDIO = (1 << 11),      // 2048
        FUNCTION_TELEMETRY_IBUS = (1 << 12),      // 4096
        FUNCTION_VTX_TRAMP = (1 << 13),           // 8192
        FUNCTION_RCDEVICE = (1 << 14),            // 16384
        FUNCTION_LIDAR_TF = (1 << 15),            // 32768
        FUNCTION_DEBUG = (1 << 16),               // 65536
    } serialPortFunction_e;

    typedef enum {
        BAUD_AUTO = 0,
        BAUD_9600,
        BAUD_19200,
        BAUD_38400,
        BAUD_57600,
        BAUD_115200,
        BAUD_230400,
        BAUD_250000,
        BAUD_400000,
        BAUD_460800,
        BAUD_500000,
        BAUD_921600,
        BAUD_1000000,
        BAUD_1500000,
        BAUD_2000000,
        BAUD_2470000
    } baudRate_e;

    extern const uint32_t baudRates[];

    // serial port identifiers are now fixed, these values are used by MSP commands.
    typedef enum {
        SERIAL_PORT_NONE = -1,
        SERIAL_PORT_USART1 = 0,
        SERIAL_PORT_USART2,
        SERIAL_PORT_USART3,
        SERIAL_PORT_UART4,
        SERIAL_PORT_UART5,
        SERIAL_PORT_USART6,
        SERIAL_PORT_USART7,
        SERIAL_PORT_USART8,
        SERIAL_PORT_USB_VCP = 20,
    } serialPortIdentifier_e;

    extern const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT];

    //
    // runtime
    //
    typedef struct serialPortUsage_s {
        serialPort_t *serialPort;     // todo: Serial Prot Config
        serialPortFunction_e function;
        serialPortIdentifier_e identifier;
    } serialPortUsage_t;

    baudRate_e lookupBaudRateIndex(uint32_t baudRate);
    serialPortUsage_t *findSerialPortUsageByIdentifier(const serialPortIdentifier_e identifier);
    serialPortUsage_t *findSerialPortUsageByPort(const serialPort_t *serialPort);
    serialPortUsage_t *findSerialPortUsageByFunction(const serialPortFunction_e function);
    void serialInit(serialPortIdentifier_e serialPortToDisable);

}  // namespace RMFC
#endif  // DEVICESERIAL_H
