/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#pragma once

#include <cstdarg>

#include "usart.h"

// 前向声明
typedef struct serialPort_s serialPort_t;
typedef enum { MODE_RX = 1 << 0, MODE_TX = 1 << 1, MODE_RXTX = MODE_RX | MODE_TX } portMode_e;
typedef enum {
    SERIAL_NOT_INVERTED = 0 << 0,
    SERIAL_INVERTED = 1 << 0,
    SERIAL_STOPBITS_1 = 0 << 1,
    SERIAL_STOPBITS_2 = 1 << 1,
    SERIAL_PARITY_NO = 0 << 2,
    SERIAL_PARITY_EVEN = 1 << 2,
    SERIAL_UNIDIR = 0 << 3,
    SERIAL_BIDIR = 1 << 3,
    /*
     * Note on SERIAL_BIDIR_PP
     * With SERIAL_BIDIR_PP, the very first start bit of back-to-back bytes
     * is lost and the first data byte will be lost by a framing error.
     * To ensure the first start bit to be sent, prepend a zero byte (0x00)
     * to actual data bytes.
     */
    SERIAL_BIDIR_OD = 0 << 4,
    SERIAL_BIDIR_PP = 1 << 4,
    SERIAL_BIDIR_NOPULL = 1 << 5,  // disable pulls in BIDIR RX mode
    SERIAL_BIDIR_PP_PD = 1 << 6,   // PP mode, normall inverted, but with PullDowns, to fix SA after
                                   // bidir issue fixed (#10220)
    // If this option is set then switch the TX line to input when not in use to detect it being
    // pulled low
    SERIAL_CHECK_TX = 1 << 7,
} portOptions_e;

// 添加回调函数类型定义
typedef void (*serial_rx_callback_t)(void* args);

struct serialPortVTable {
    void (*serialWrite)(serialPort_t* instance, uint8_t ch);
    uint32_t (*serialTotalRxWaiting)(const serialPort_t* instance);
    uint32_t (*serialTotalTxFree)(const serialPort_t* instance);
    uint32_t (*serialRead)(serialPort_t* instance, uint8_t** data);
    void (*serialSetBaudRate)(serialPort_t* instance, uint32_t baudRate);
    bool (*isSerialTransmitBufferEmpty)(const serialPort_t* instance);
    void (*setMode)(serialPort_t* instance, portMode_e mode);
    void (*writeBuf)(serialPort_t* instance, const void* data, int length);
    void (*beginWrite)(serialPort_t* instance);
    void (*endWrite)(serialPort_t* instance);
    void (*registerCallback)(serialPort_t* instance, serial_rx_callback_t callback, void* args);
};

// 串口结构体
typedef struct serialPort_s {
    const struct serialPortVTable* vTable;
    uint8_t identifier;
    portMode_e mode;
    portOptions_e options;
    uint32_t baudRate;
    serial_rx_callback_t rxCallback;
    void* rxCallbackData;
    void* privateData;
} serialPort_t;

#define TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK \
    (FUNCTION_TELEMETRY_FRSKY_HUB | FUNCTION_TELEMETRY_LTM | FUNCTION_TELEMETRY_MAVLINK)
#define TELEMETRY_PORT_FUNCTIONS_MASK                                    \
    (TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK | FUNCTION_TELEMETRY_HOTT | \
     FUNCTION_TELEMETRY_SMARTPORT)

// 公共接口函数
void serialWrite(serialPort_t* instance, uint8_t ch);
void serialWriteBuf(serialPort_t* instance, const uint8_t* data, int length);
uint32_t serialRxBytesWaiting(const serialPort_t* instance);
uint32_t serialTxBytesFree(const serialPort_t* instance);
template <bool FromISR = false>
uint32_t serialRead(serialPort_t* instance, uint8_t** data);
void serialSetBaudRate(serialPort_t* instance, uint32_t baudRate);
bool isSerialTransmitBufferEmpty(const serialPort_t* instance);
void serialSetMode(serialPort_t* instance, portMode_e mode);
void serialBeginWrite(serialPort_t* instance);
void serialEndWrite(serialPort_t* instance);
uint32_t serialGetBaudRate(serialPort_t* instance);
// 添加RegisterCallback公共接口函数声明
void SerialRegisterCallback(serialPort_t* instance, serial_rx_callback_t callback, void* args);

// 辅助函数
void serialPrint(serialPort_t* instance, const char* str);
uint32_t serialPrintFormatted(serialPort_t* instance, const char* format, ...);
uint32_t serialPrintlnFormatted(serialPort_t* instance, const char* format, ...);

// 工厂函数
serialPort_t* newSerialPort(UART_HandleTypeDef* huart, uint32_t baudRate, portMode_e mode,
                            portOptions_e options, serial_rx_callback_t callback, void* args);
serialPort_t* newSerialPortUSB(serial_rx_callback_t callback, void* args);
void deleteSerialPort(serialPort_t* instance);

// 用于缓冲写入的辅助函数
void serialWriteBufShim(void* instance, const uint8_t* data, int length);
