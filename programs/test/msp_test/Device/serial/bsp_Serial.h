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

#include "usart.h"
#include <cstdarg>

// 前向声明
typedef struct serialPort_s serialPort_t;
typedef enum { MODE_RX = 1 << 0, MODE_TX = 1 << 1, MODE_RXTX = MODE_RX | MODE_TX } portMode_e;
typedef enum { SERIAL_NOT_INVERTED = 0, SERIAL_INVERTED = 1 << 0 } portOptions_e;
typedef void (*serialReceiveCallbackPtr)(
    uint16_t data, void* rxCallbackData);  // used by serial drivers to return frames to app

// 虚函数表，定义了串口的各种操作
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
};

// 串口结构体
typedef struct serialPort_s {
    const struct serialPortVTable* vTable;
    uint8_t identifier;
    portMode_e mode;
    portOptions_e options;
    uint32_t baudRate;
    serialReceiveCallbackPtr rxCallback;
    void* rxCallbackData;
    void* privateData;
} serialPort_t;

// 公共接口函数
void serialWrite(serialPort_t* instance, uint8_t ch);
void serialWriteBuf(serialPort_t* instance, const uint8_t* data, int length);
uint32_t serialRxBytesWaiting(const serialPort_t* instance);
uint32_t serialTxBytesFree(const serialPort_t* instance);
uint32_t serialRead(serialPort_t* instance, uint8_t** data);
void serialSetBaudRate(serialPort_t* instance, uint32_t baudRate);
bool isSerialTransmitBufferEmpty(const serialPort_t* instance);
void serialSetMode(serialPort_t* instance, portMode_e mode);
void serialBeginWrite(serialPort_t* instance);
void serialEndWrite(serialPort_t* instance);
uint32_t serialGetBaudRate(serialPort_t* instance);

// 辅助函数
void serialPrint(serialPort_t* instance, const char* str);
uint32_t serialPrintFormatted(serialPort_t* instance, const char* format, ...);
uint32_t serialPrintlnFormatted(serialPort_t* instance, const char* format, ...);

// 工厂函数
serialPort_t* newSerialPort(UART_HandleTypeDef* huart, uint32_t baudRate);
serialPort_t* newSerialPortUSB();
void deleteSerialPort(serialPort_t* instance);

// 用于缓冲写入的辅助函数
void serialWriteBufShim(void* instance, const uint8_t* data, int length);
