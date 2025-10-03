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

#include "bsp_print.h"
#include "bsp_uart.h"
#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "target.h"

namespace RMFC {

    const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT] = {
#ifdef USEUSB_VCP
        SERIAL_PORT_USB_VCP,
#endif
#ifdef USEUART_1
        SERIAL_PORT_USART1,
#endif
#ifdef USEUART_2
        SERIAL_PORT_USART2,
#endif
#ifdef USEUART_3
        SERIAL_PORT_USART3,
#endif
#ifdef USEUART_4
        SERIAL_PORT_UART4,
#endif
#ifdef USEUART_5
        SERIAL_PORT_UART5,
#endif
#ifdef USEUART_6
        SERIAL_PORT_USART6,
#endif
#ifdef USEUART_7
        SERIAL_PORT_USART7,
#endif
#ifdef USEUART_8
        SERIAL_PORT_USART8,
#endif
    };

    static uint8_t serialPortCount;

    static serialPortUsage_t serialPortUsageList[SERIAL_PORT_COUNT];

    const uint32_t baudRates[] = {0,       9600,    19200,   38400,  57600,  115200,
                                  230400,  250000,  400000,  460800, 500000, 921600,
                                  1000000, 1500000, 2000000, 2470000};  // see baudRate_e

#define BAUD_RATE_COUNT (sizeof(baudRates) / sizeof(baudRates[0]))

    //
    // Serial Port PG
    //

    PG_REGISTER_WITH_RESET_FN(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 0);

    void pgResetFn_serialConfig(serialConfig_t* serialConfig, int size) {
        UNUSED(size);
        memset(serialConfig, 0, sizeof(serialConfig_t));

        for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
            serialConfig->portConfigs[i].identifier = serialPortIdentifiers[i];
            serialConfig->portConfigs[i].msp_baudrateIndex = BAUD_115200;
            serialConfig->portConfigs[i].navigation_baudrateIndex = BAUD_57600;
            serialConfig->portConfigs[i].referee_baudrateIndex = BAUD_AUTO;
            serialConfig->portConfigs[i].blackbox_baudrateIndex = BAUD_115200;
        }

        serialConfig->portConfigs[0].functionMask = FUNCTION_MSP;

#ifdef SERIALRX_UART
        serialPortConfig_t* serialRxUartConfig = serialFindPortConfiguration(SERIALRX_UART);
        if (serialRxUartConfig) {
            serialRxUartConfig->functionMask = FUNCTION_RX_SERIAL;
        }
#endif

#ifdef SBUS_TELEMETRY_UART
        serialPortConfig_t* serialTlemetryUartConfig =
            serialFindPortConfiguration(SBUS_TELEMETRY_UART);
        if (serialTlemetryUartConfig) {
            serialTlemetryUartConfig->functionMask = FUNCTION_TELEMETRY_SMARTPORT;
        }
#endif

#if defined(USEUSB_VCP) && defined(USEUART)
        if (serialConfig->portConfigs[0].identifier == SERIAL_PORT_USB_VCP) {
#if defined(MSP_FUNCTION_UART)
            serialPortConfig_t* uart1Config = serialFindPortConfiguration(MSP_FUNCTION_UART);
#else
            serialPortConfig_t* uart1Config = serialFindPortConfiguration(SERIAL_PORT_USART1);
#endif
            if (uart1Config) {
                uart1Config->functionMask = FUNCTION_MSP;
            }
        }
#endif

        serialConfig->reboot_character = 'R';
        serialConfig->serial_update_rate_hz = 100;

#if defined(DEBUG_PG)
        print("Serial config reset completed\r\n");
#endif
    }

    //
    // Serial Port Usage
    //

    baudRate_e getBaudrateIndexByFunction(const serialPortConfig_t* portConfig,
                                          baudrateFunctionIndex_e baudrateFunctionIndex) {
        switch (baudrateFunctionIndex) {
            case BLACKBOX_BAUD_Index:
                return static_cast<baudRate_e>(portConfig->blackbox_baudrateIndex);
            case MSP_BAUD_Index:
                return static_cast<baudRate_e>(portConfig->msp_baudrateIndex);
            case NAVIGATION_BAUD_Index:
                return static_cast<baudRate_e>(portConfig->navigation_baudrateIndex);
            case REREFEE_BAUD_Index:
                return static_cast<baudRate_e>(portConfig->referee_baudrateIndex);
            default:
                return BAUD_AUTO;
        }
    }

    baudRate_e lookupBaudRateIndex(uint32_t baudRate) {
        for (uint8_t index = 0; index < BAUD_RATE_COUNT; index++) {
            if (baudRates[index] == baudRate) {
                return static_cast<baudRate_e>(index);
            }
        }
        return BAUD_AUTO;
    }

    serialPortUsage_t* findSerialPortUsageByIdentifier(const serialPortIdentifier_e identifier) {
        uint8_t index;
        for (index = 0; index < SERIAL_PORT_COUNT; index++) {
            serialPortUsage_t* candidate = &serialPortUsageList[index];
            if (candidate->identifier == identifier) {
                return candidate;
            }
        }
        return NULL;
    }

    serialPortUsage_t* findSerialPortUsageByPort(const serialPort_t* serialPort) {
        uint8_t index;
        for (index = 0; index < SERIAL_PORT_COUNT; index++) {
            serialPortUsage_t* candidate = &serialPortUsageList[index];
            if (candidate->serialPort == serialPort) {
                return candidate;
            }
        }
        return NULL;
    }

    typedef struct findSerialPortConfigState_s {
        uint8_t lastIndex;
    } findSerialPortConfigState_t;

    static findSerialPortConfigState_t findSerialPortConfigState;

    serialPortConfig_t* findSerialPortConfig(serialPortFunction_e function) {
        memset(&findSerialPortConfigState, 0, sizeof(findSerialPortConfigState));

        return findNextSerialPortConfig(function);
    }

    serialPortConfig_t* findNextSerialPortConfig(serialPortFunction_e function) {
        while (findSerialPortConfigState.lastIndex < SERIAL_PORT_COUNT) {
            serialPortConfig_t* candidate =
                &serialConfigMutable()->portConfigs[findSerialPortConfigState.lastIndex++];

            if (candidate->functionMask & function) {
                return candidate;
            }
        }
        return NULL;
    }

    typedef struct findSharedSerialPortState_s {
        uint8_t lastIndex;
    } findSharedSerialPortState_t;

    portSharing_e determinePortSharing(const serialPortConfig_t* portConfig,
                                       serialPortFunction_e function) {
        if (!portConfig || (portConfig->functionMask & function) == 0) {
            return PORTSHARING_UNUSED;
        }
        return portConfig->functionMask == function ? PORTSHARING_NOT_SHARED : PORTSHARING_SHARED;
    }

    bool isSerialPortShared(const serialPortConfig_t* portConfig, uint16_t functionMask,
                            serialPortFunction_e sharedWithFunction) {
        return (portConfig) && (portConfig->functionMask & sharedWithFunction) &&
               (portConfig->functionMask & functionMask);
    }

    static findSharedSerialPortState_t findSharedSerialPortState;

    serialPort_t* findSharedSerialPort(uint16_t functionMask,
                                       serialPortFunction_e sharedWithFunction) {
        memset(&findSharedSerialPortState, 0, sizeof(findSharedSerialPortState));

        return findNextSharedSerialPort(functionMask, sharedWithFunction);
    }

    serialPort_t* findNextSharedSerialPort(uint16_t functionMask,
                                           serialPortFunction_e sharedWithFunction) {
        while (findSharedSerialPortState.lastIndex < SERIAL_PORT_COUNT) {
            const serialPortConfig_t* candidate =
                &serialConfig()->portConfigs[findSharedSerialPortState.lastIndex++];

            if (isSerialPortShared(candidate, functionMask, sharedWithFunction)) {
                const serialPortUsage_t* serialPortUsage = findSerialPortUsageByIdentifier(
                    static_cast<serialPortIdentifier_e>(candidate->identifier));
                if (!serialPortUsage) {
                    continue;
                }
                return serialPortUsage->serialPort;
            }
        }
        return NULL;
    }

#ifdef USE_TELEMETRY
#define ALL_FUNCTIONS_SHARABLE_WITH_MSP (FUNCTION_BLACKBOX | TELEMETRY_PORT_FUNCTIONS_MASK)
#else
#define ALL_FUNCTIONS_SHARABLE_WITH_MSP (FUNCTION_BLACKBOX)
#endif

    bool isSerialConfigValid(const serialConfig_t* serialConfigToCheck) {
        UNUSED(serialConfigToCheck);
        /*
         * rules:
         * - 1 MSP port minimum, max MSP ports is defined and must be adhered to.
         * - MSP is allowed to be shared with EITHER any telemetry OR blackbox.
         *   (using either / or, switching based on armed / disarmed or an AUX channel if
         * 'telemetry_switch' is true
         * - serial RX and FrSky / LTM / MAVLink telemetry can be shared
         *   (serial RX using RX line, telemetry using TX line)
         * - No other sharing combinations are valid.
         */
        uint8_t mspPortCount = 0;

        for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
            const serialPortConfig_t* portConfig = &serialConfigToCheck->portConfigs[index];

            if (portConfig->functionMask & FUNCTION_MSP) {
                mspPortCount++;
            }

            uint8_t bitCount = BITCOUNT(portConfig->functionMask);
            if (bitCount > 1) {
                // shared
                if (bitCount > 2) {
                    return false;
                }

                if ((portConfig->functionMask & FUNCTION_MSP) &&
                    (portConfig->functionMask & ALL_FUNCTIONS_SHARABLE_WITH_MSP)) {
                    // MSP & telemetry
#ifdef USE_TELEMETRY
                } else if (telemetryCheckRxPortShared(portConfig)) {
                    // serial RX & telemetry
#endif
                } else {
                    // some other combination
                    return false;
                }
            }
        }

        if (mspPortCount == 0 || mspPortCount > MAX_MSP_PORT_COUNT) {
            return false;
        }
        return true;
    }

    serialPortConfig_t* serialFindPortConfiguration(serialPortIdentifier_e identifier) {
        for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
            serialPortConfig_t* candidate = &serialConfigMutable()->portConfigs[index];
            if (candidate->identifier == identifier) {
                return candidate;
            }
        }
        return NULL;
    }

    bool doesConfigurationUsePort(serialPortIdentifier_e identifier) {
        serialPortConfig_t* candidate = serialFindPortConfiguration(identifier);
        return candidate != NULL && candidate->functionMask;
    }

    serialPort_t* openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e function,
                                 serial_rx_callback_t rxCallback, void* rxCallbackData,
                                 uint32_t baudRate, portMode_e mode, portOptions_e options,
                                 uartSetup_t* uartSetup) {
#if !(defined(USE_UART) || defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2))
        UNUSED(rxCallback);
        UNUSED(rxCallbackData);
        UNUSED(baudRate);
        UNUSED(mode);
        UNUSED(options);
        UNUSED(uartSetup);
#endif

        serialPortUsage_t* serialPortUsage = findSerialPortUsageByIdentifier(identifier);
        if (!serialPortUsage || serialPortUsage->function != FUNCTION_NONE) {
            // not available / already in use
            return nullptr;
        }

        serialPort_t* serialPort = nullptr;

        switch (identifier) {
#ifdef USEUSB_VCP
            case SERIAL_PORT_USB_VCP:
                serialPort = newSerialPortUSB(rxCallback, rxCallbackData, uartSetup);
                break;
#endif

#if defined(USEUART)
#ifdef USEUART_1
            case SERIAL_PORT_USART1:
                serialPort = newSerialPort(USEUART_1, baudRate, mode, options, rxCallback,
                                           rxCallbackData, uartSetup);
                break;
#endif
#ifdef USEUART_2
            case SERIAL_PORT_USART2:
                serialPort = newSerialPort(USEUART_2, baudRate, mode, options, rxCallback,
                                           rxCallbackData, uartSetup);
                break;
#endif
#ifdef USEUART_3
            case SERIAL_PORT_USART3:
                serialPort = newSerialPort(USEUART_3, baudRate, mode, options, rxCallback,
                                           rxCallbackData, uartSetup);
                break;
#endif
#ifdef USEUART_4
            case SERIAL_PORT_UART4:
                serialPort = newSerialPort(USEUART_4, baudRate, mode, options, rxCallback,
                                           rxCallbackData, uartSetup);
                break;
#endif
#ifdef USEUART_5
            case SERIAL_PORT_UART5:
                serialPort = newSerialPort(USEUART_5, baudRate, mode, options, rxCallback,
                                           rxCallbackData, uartSetup);
                break;
#endif
#endif
#ifdef USEUART_6
            case SERIAL_PORT_USART6:
                serialPort = newSerialPort(USEUART_6, baudRate, mode, options, rxCallback,
                                           rxCallbackData, uartSetup);
                break;
#endif
#ifdef USEUART_7
            case SERIAL_PORT_USART7:
                serialPort = newSerialPort(USEUART_7, baudRate, mode, options, rxCallback,
                                           rxCallbackData, uartSetup);
                break;
#endif
#ifdef USEUART_8
            case SERIAL_PORT_USART8:
                serialPort = newSerialPort(USEUART_8, baudRate, mode, options, rxCallback,
                                           rxCallbackData, uartSetup);
                break;
#endif
            default:
                break;
        }

        if (!serialPort) {
            return nullptr;
        }

        serialPort->identifier = identifier;

        serialPortUsage->function = function;
        serialPortUsage->serialPort = serialPort;

        return serialPort;
    }

    void closeSerialPort(serialPort_t* serialPort) {
        serialPortUsage_t* serialPortUsage = findSerialPortUsageByPort(serialPort);
        if (!serialPortUsage) {
            // already closed
            return;
        }

        // TODO wait until data has been transmitted.

        serialPort->rxCallback = nullptr;

        serialPortUsage->function = FUNCTION_NONE;
        serialPortUsage->serialPort = nullptr;
    }

    void serialInit(bool softserialEnabled, serialPortIdentifier_e serialPortToDisable) {
#if !defined(USE_SOFTSERIAL1) && !defined(USE_SOFTSERIAL2)
        UNUSED(softserialEnabled);
#endif

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

    void serialRemovePort(serialPortIdentifier_e identifier) {
        for (uint8_t index = 0; index < SERIAL_PORT_COUNT; index++) {
            if (serialPortUsageList[index].identifier == identifier) {
                serialPortUsageList[index].identifier = SERIAL_PORT_NONE;
                serialPortCount--;
            }
        }
    }

    uint8_t serialGetAvailablePortCount(void) {
        return serialPortCount;
    }

    bool serialIsPortAvailable(serialPortIdentifier_e identifier) {
        for (uint8_t index = 0; index < SERIAL_PORT_COUNT; index++) {
            if (serialPortUsageList[index].identifier == identifier) {
                return true;
            }
        }
        return false;
    }

    void waitForSerialPortToFinishTransmitting(serialPort_t* serialPort) {
        while (!isSerialTransmitBufferEmpty(serialPort)) {
            osDelay(10);
        };
    }

#if defined(USE_GPS) || !defined(SKIP_SERIAL_PASSTHROUGH)
    // Default data consumer for serialPassThrough.
    static void nopConsumer(uint8_t data) {
        UNUSED(data);
    }

    /*
     A high-level serial passthrough implementation. Used by cli to start an
     arbitrary serial passthrough "proxy". Optional callbacks can be given to allow
     for specialized data processing.
     */
    void serialPassthrough(serialPort_t* left, serialPort_t* right, serialConsumer* leftC,
                           serialConsumer* rightC) {
        waitForSerialPortToFinishTransmitting(left);
        waitForSerialPortToFinishTransmitting(right);

        if (!leftC)
            leftC = &nopConsumer;
        if (!rightC)
            rightC = &nopConsumer;

        // LED0_OFF;
        // LED1_OFF;

        // Either port might be open in a mode other than MODE_RXTX. We rely on
        // serialRxBytesWaiting() to do the right thing for a TX only port. No
        // special handling is necessary OR performed.
        while (1) {
            // TODO: maintain a timestamp of last data received. Use this to
            // implement a guard interval and check for `+++` as an escape sequence
            // to return to CLI command mode.
            // https://en.wikipedia.org/wiki/Escape_sequence#Modem_control
            if (serialRxBytesWaiting(left)) {
                // LED0_ON;
                uint8_t* data;
                uint32_t len = serialRead(left, &data);
                for (uint32_t i = 0; i < len; i++) {
                    serialWrite(right, data[i]);
                    leftC(data[i]);
                }
                // LED0_OFF;
            }
            if (serialRxBytesWaiting(right)) {
                // LED0_ON;
                uint8_t* data;
                uint32_t len = serialRead(right, &data);
                for (uint32_t i = 0; i < len; i++) {
                    serialWrite(left, data[i]);
                    rightC(data[i]);
                }
                // LED0_OFF;
            }
        }
    }
#endif

}  // namespace RMFC
