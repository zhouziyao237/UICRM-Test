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
#include "bsp_PG/bsp_pg.h"
#include "bsp_serial.h"
#include "bsp_uart.h"
#include "bsp_usb.h"

#define SERIAL_PORT_IDENTIFIER_TO_INDEX(x) \
    (((x) <= SERIAL_PORT_USART8) ? (x) : (RESOURCE_SOFT_OFFSET + ((x) - SERIAL_PORT_SOFTSERIAL1)))
#define SERIAL_PORT_IDENTIFIER_TO_UARTDEV(x) ((x) - SERIAL_PORT_USART1 + UARTDEV_1)
#define BIT(x) (1 << (x))
#define BITCOUNT(x) (((BX_(x) + (BX_(x) >> 4)) & 0x0F0F0F0F) % 255)
#define BX_(x) \
    ((x) - (((x) >> 1) & 0x77777777) - (((x) >> 2) & 0x33333333) - (((x) >> 3) & 0x11111111))

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
        FUNCTION_RFS_MSP = (1 << 3),              // 8
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
        FUNCTION_FRSKY_OSD = (1 << 16),           // 65536
        FUNCTION_VTX_MSP = (1 << 17),             // 131072
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
        BAUD_2470000,
        BAUD_COUNT
    } baudRate_e;

    extern const uint32_t baudRates[];

    // serial port identifiers are now fixed, these values are used by MSP commands.
    typedef enum {
        SERIAL_PORT_ALL = -2,
        SERIAL_PORT_NONE = -1,
        SERIAL_PORT_USART1 = 0,
        SERIAL_PORT_USART2,
        SERIAL_PORT_USART3,
        SERIAL_PORT_UART4,
        SERIAL_PORT_UART5,
        SERIAL_PORT_USART6,
        SERIAL_PORT_USART7,
        SERIAL_PORT_USART8,
        SERIAL_PORT_UART9,
        SERIAL_PORT_USART10,
        SERIAL_PORT_USB_VCP = 20,
        SERIAL_PORT_SOFTSERIAL1 = 30,
        SERIAL_PORT_SOFTSERIAL2,
        SERIAL_PORT_LPUART1 = 40,
        SERIAL_PORT_IDENTIFIER_MAX = SERIAL_PORT_LPUART1,
    } serialPortIdentifier_e;

    extern const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT];

    //
    // runtime
    //
    typedef struct serialPortUsage_s {
        serialPort_t* serialPort;  // todo: Serial Prot Config
        serialPortFunction_e function;
        serialPortIdentifier_e identifier;
    } serialPortUsage_t;

    serialPort_t* findSharedSerialPort(uint16_t functionMask,
                                       serialPortFunction_e sharedWithFunction);
    serialPort_t* findNextSharedSerialPort(uint16_t functionMask,
                                           serialPortFunction_e sharedWithFunction);

    //
    // configuration
    //
    typedef void serialConsumer(uint8_t);

    typedef struct serialPortConfig_s {
        uint32_t functionMask;
        int8_t identifier;
        uint8_t msp_baudrateIndex;
        uint8_t navigation_baudrateIndex;
        uint8_t blackbox_baudrateIndex;
        uint8_t referee_baudrateIndex;  // not used for all telemetry systems, e.g. HoTT only
                                        // works at 19200.
    } serialPortConfig_t;

    typedef struct serialConfig_s {
        serialPortConfig_t portConfigs[SERIAL_PORT_COUNT];
        uint16_t serial_update_rate_hz;
        uint8_t reboot_character;  // which byte is used to reboot. Default 'R', could be changed
                                   // carefully to something else.
    } serialConfig_t;

    PG_DECLARE(serialConfig_t, serialConfig);

    //
    // configuration
    //

    void pgResetFn_serialConfig(serialConfig_t* serialConfig,
                                int size);  //!!TODO remove need for this

    void serialInit(bool softserialEnabled, serialPortIdentifier_e serialPortToDisable);
    void serialRemovePort(serialPortIdentifier_e identifier);
    uint8_t serialGetAvailablePortCount(void);
    bool serialIsPortAvailable(serialPortIdentifier_e identifier);
    bool isSerialConfigValid(const serialConfig_t* serialConfig);
    serialPortConfig_t* serialFindPortConfiguration(serialPortIdentifier_e identifier);
    bool doesConfigurationUsePort(serialPortIdentifier_e portIdentifier);
    serialPortConfig_t* findSerialPortConfig(serialPortFunction_e function);
    serialPortConfig_t* findNextSerialPortConfig(serialPortFunction_e function);
    portSharing_e determinePortSharing(const serialPortConfig_t* portConfig,
                                       serialPortFunction_e function);
    bool isSerialPortShared(const serialPortConfig_t* portConfig, uint16_t functionMask,
                            serialPortFunction_e sharedWithFunction);

    serialPortUsage_t* findSerialPortUsageByIdentifier(serialPortIdentifier_e identifier);
    int findSerialPortIndexByIdentifier(serialPortIdentifier_e identifier);

    typedef enum {
        MSP_BAUD_Index = 0,
        NAVIGATION_BAUD_Index = 1,
        BLACKBOX_BAUD_Index = 2,
        REREFEE_BAUD_Index = 3,
    } baudrateFunctionIndex_e;

    baudRate_e getBaudrateIndexByFunction(const serialPortConfig_t* serialConfig,
                                          baudrateFunctionIndex_e baudrateFunctionIndex);

    //
    // runtime
    //
    serialPort_t* openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e function,
                                 serial_rx_callback_t rxCallback, void* rxCallbackData,
                                 uint32_t baudrate, portMode_e mode, portOptions_e options);
    void closeSerialPort(serialPort_t* serialPort);
    void waitForSerialPortToFinishTransmitting(serialPort_t* serialPort);
    baudRate_e lookupBaudRateIndex(uint32_t baudRate);

    //
    // msp/cli/bootloader
    //
    void serialPassthrough(serialPort_t* left, serialPort_t* right, serialConsumer* leftC,
                           serialConsumer* rightC);

}  // namespace RMFC
#endif  // DEVICESERIAL_H
