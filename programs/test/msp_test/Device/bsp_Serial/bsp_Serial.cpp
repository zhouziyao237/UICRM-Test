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

#include "bsp_Serial.h"

#include <stdlib.h>
#include <string.h>

#include <cstdarg>

#include "Target.h"
#include "bsp_uart.h"
#include "bsp_usb.h"

#define MAX_PRINT_LEN 1024
#define RX_SIGNAL (1 << 0)

// UART私有数据结构
typedef struct {
    bsp::UART* uart_instance;
} uartPrivateData_t;

// USB私有数据结构
typedef struct {
    bsp::VirtualUSB* usb_instance;
} usbPrivateData_t;

// VTable函数实现
static void uartSerialWrite(serialPort_t* instance, uint8_t ch) {
    uartPrivateData_t* privateData = (uartPrivateData_t*)instance->privateData;
    if (privateData && privateData->uart_instance) {
        privateData->uart_instance->Write(&ch, 1);
    }
}

static uint32_t uartSerialTotalRxWaiting(const serialPort_t* instance) {
    // UART实现中无法直接获取等待的字节数，这里返回0
    UNUSED(instance);
    return 0;
}

static uint32_t uartSerialTotalTxFree(const serialPort_t* instance) {
    // 假设缓冲区总是有足够的空间
    UNUSED(instance);
    return 1024;
}

static uint32_t uartSerialRead(serialPort_t* instance, uint8_t** data) {
    uartPrivateData_t* privateData = (uartPrivateData_t*)instance->privateData;
    if (privateData && privateData->uart_instance) {
        uint32_t len = privateData->uart_instance->Read(data);
        if (len > 0) {
            return len;
        }
    }
    return 0;
}

static void uartSerialSetBaudRate(serialPort_t* instance, uint32_t baudRate) {
    // 在当前实现中无法动态更改波特率
    instance->baudRate = baudRate;
}

static bool uartIsSerialTransmitBufferEmpty(const serialPort_t* instance) {
    // 假设传输缓冲区总是空的
    UNUSED(instance);
    return true;
}

static void uartSetMode(serialPort_t* instance, portMode_e mode) {
    instance->mode = mode;
}

static void uartWriteBuf(serialPort_t* instance, const void* data, int length) {
    uartPrivateData_t* privateData = (uartPrivateData_t*)instance->privateData;
    if (privateData && privateData->uart_instance) {
        privateData->uart_instance->Write((const uint8_t*)data, length);
    }
}

static void uartBeginWrite(serialPort_t* instance) {
    // UART实现中不需要特殊处理
    UNUSED(instance);
}

static void uartEndWrite(serialPort_t* instance) {
    // UART实现中不需要特殊处理
    UNUSED(instance);
}

// UART回调注册函数实现
static void uartRegisterCallback(serialPort_t* instance, serial_rx_callback_t callback,
                                 void* args) {
    uartPrivateData_t* privateData = (uartPrivateData_t*)instance->privateData;
    if (privateData && privateData->uart_instance) {
        privateData->uart_instance->RegisterCallback(callback, args);
    }
}

// USB VTable函数实现
static void usbSerialWrite(serialPort_t* instance, uint8_t ch) {
    usbPrivateData_t* privateData = (usbPrivateData_t*)instance->privateData;
    if (privateData && privateData->usb_instance) {
        privateData->usb_instance->Write(&ch, 1);
    }
}

static uint32_t usbSerialTotalRxWaiting(const serialPort_t* instance) {
    // USB实现中无法直接获取等待的字节数，这里返回0
    UNUSED(instance);
    return 0;
}

static uint32_t usbSerialTotalTxFree(const serialPort_t* instance) {
    // 假设缓冲区总是有足够的空间
    UNUSED(instance);
    return 1024;
}

static uint32_t usbSerialRead(serialPort_t* instance, uint8_t** data) {
    usbPrivateData_t* privateData = (usbPrivateData_t*)instance->privateData;
    if (privateData && privateData->usb_instance) {
        uint32_t len = privateData->usb_instance->Read(data);
        if (len > 0) {
            return len;
        }
    }
    return 0;
}

static void usbSerialSetBaudRate(serialPort_t* instance, uint32_t baudRate) {
    // USB虚拟串口通常不需要设置波特率
    instance->baudRate = baudRate;
}

static bool usbIsSerialTransmitBufferEmpty(const serialPort_t* instance) {
    // 假设传输缓冲区总是空的
    UNUSED(instance);
    return true;
}

static void usbSetMode(serialPort_t* instance, portMode_e mode) {
    instance->mode = mode;
}

static void usbWriteBuf(serialPort_t* instance, const void* data, int count) {
    usbPrivateData_t* privateData = (usbPrivateData_t*)instance->privateData;
    if (privateData && privateData->usb_instance) {
        privateData->usb_instance->Write((uint8_t*)data, count);
    }
}

static void usbBeginWrite(serialPort_t* instance) {
    // USB实现中不需要特殊处理
    UNUSED(instance);
}

static void usbEndWrite(serialPort_t* instance) {
    // USB实现中不需要特殊处理
    UNUSED(instance);
}

// USB回调注册函数实现
static void usbRegisterCallback(serialPort_t* instance, serial_rx_callback_t callback, void* args) {
    usbPrivateData_t* privateData = (usbPrivateData_t*)instance->privateData;
    if (privateData && privateData->usb_instance) {
        privateData->usb_instance->RegisterCallback(callback, args);
    }
}

// UART VTable
static const struct serialPortVTable uartVTable = {
    uartSerialWrite,       uartSerialTotalRxWaiting,
    uartSerialTotalTxFree, uartSerialRead,
    uartSerialSetBaudRate, uartIsSerialTransmitBufferEmpty,
    uartSetMode,           uartWriteBuf,
    uartBeginWrite,        uartEndWrite,
    uartRegisterCallback};

// USB VTable
static const struct serialPortVTable usbVTable = {
    usbSerialWrite,       usbSerialTotalRxWaiting,
    usbSerialTotalTxFree, usbSerialRead,
    usbSerialSetBaudRate, usbIsSerialTransmitBufferEmpty,
    usbSetMode,           usbWriteBuf,
    usbBeginWrite,        usbEndWrite,
    usbRegisterCallback};

// 公共接口函数实现
void serialWrite(serialPort_t* instance, uint8_t ch) {
    instance->vTable->serialWrite(instance, ch);
}

uint32_t serialGetBaudRate(serialPort_t* instance) {
    return instance ? instance->baudRate : 0;
}

// 添加RegisterCallback公共接口函数实现
void serialRegisterCallback(serialPort_t* instance, serial_rx_callback_t callback, void* args) {
    if (instance->vTable->registerCallback) {
        instance->vTable->registerCallback(instance, callback, args);
    }
}

void serialWriteBuf(serialPort_t* instance, const uint8_t* data, int length) {
    if (instance->vTable->writeBuf) {
        instance->vTable->writeBuf(instance, data, length);
    } else {
        for (const uint8_t* p = data; length > 0; length--, p++) {
            serialWrite(instance, *p);
        }
    }
}

uint32_t serialRxBytesWaiting(const serialPort_t* instance) {
    return instance->vTable->serialTotalRxWaiting(instance);
}

uint32_t serialTxBytesFree(const serialPort_t* instance) {
    return instance->vTable->serialTotalTxFree(instance);
}

uint32_t serialRead(serialPort_t* instance, uint8_t** data) {
    return instance->vTable->serialRead(instance, data);
}

void serialSetBaudRate(serialPort_t* instance, uint32_t baudRate) {
    instance->vTable->serialSetBaudRate(instance, baudRate);
}

bool isSerialTransmitBufferEmpty(const serialPort_t* instance) {
    return instance->vTable->isSerialTransmitBufferEmpty(instance);
}

void serialSetMode(serialPort_t* instance, portMode_e mode) {
    instance->vTable->setMode(instance, mode);
}

void serialBeginWrite(serialPort_t* instance) {
    if (instance->vTable->beginWrite) {
        instance->vTable->beginWrite(instance);
    }
}

void serialEndWrite(serialPort_t* instance) {
    if (instance->vTable->endWrite) {
        instance->vTable->endWrite(instance);
    }
}

void serialPrint(serialPort_t* instance, const char* str) {
    if (!instance || !str)
        return;

    uint8_t ch;
    while ((ch = *(str++)) != 0) {
        serialWrite(instance, ch);
    }
}

uint32_t serialPrintFormatted(serialPort_t* instance, const char* format, ...) {
    if (!instance || !format)
        return 0;

#ifdef NDEBUG
    UNUSED(format);
    return 0;
#else
    char buffer[256];  // 根据需要调整缓冲区大小
    va_list args;

    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length > 0) {
        serialWriteBuf(instance, (uint8_t*)buffer, length);
        return length;
    }
    return 0;
#endif
}

uint32_t serialPrintlnFormatted(serialPort_t* instance, const char* format, ...) {
    if (!instance || !format)
        return 0;

#ifdef NDEBUG
    UNUSED(format);
    return 0;
#else
    char buffer[256];  // 根据需要调整缓冲区大小
    va_list args;

    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length > 0) {
        // 添加换行符
        if (length < (int)sizeof(buffer) - 2) {  // 确保有足够的空间
            buffer[length] = '\r';
            buffer[length + 1] = '\n';
            length += 2;
        }

        serialWriteBuf(instance, (uint8_t*)buffer, length);
        return length;
    }
    return 0;
#endif
}

void serialWriteBufShim(void* instance, const uint8_t* data, int length) {
    serialWriteBuf((serialPort_t*)instance, data, length);
}

// 创建UART串口实例的工厂函数
serialPort_t* newSerialPort(UART_HandleTypeDef* huart, uint32_t baudRate, portMode_e mode,
                            portOptions_e options, serial_rx_callback_t callback, void* args) {
    if (!huart)
        return NULL;

    serialPort_t* instance = (serialPort_t*)malloc(sizeof(serialPort_t));
    if (!instance)
        return NULL;

    memset(instance, 0, sizeof(serialPort_t));

    uartPrivateData_t* privateData = (uartPrivateData_t*)malloc(sizeof(uartPrivateData_t));
    if (!privateData) {
        free(instance);
        return NULL;
    }

    privateData->uart_instance = new bsp::UART(huart);
    if (!privateData->uart_instance) {
        free(privateData);
        free(instance);
        return NULL;
    }

    // 初始化UART
    privateData->uart_instance->SetupTx(MAX_PRINT_LEN * 2, true);
    privateData->uart_instance->SetupRx(MAX_PRINT_LEN * 2, true);
    if (baudRate != huart->Init.BaudRate) {
        privateData->uart_instance->SetBaudrate(baudRate);
    }

    privateData->uart_instance->RegisterCallback(callback, args);

    // 设置VTable和属性
    instance->vTable = &uartVTable;
    instance->baudRate = baudRate;
    instance->mode = mode;
    instance->options = options;
    instance->privateData = privateData;

    return instance;
}

// 创建USB串口实例的工厂函数
serialPort_t* newSerialPortUSB(serial_rx_callback_t callback, void* args) {
    serialPort_t* instance = (serialPort_t*)malloc(sizeof(serialPort_t));
    if (!instance)
        return NULL;

    memset(instance, 0, sizeof(serialPort_t));

    usbPrivateData_t* privateData = (usbPrivateData_t*)malloc(sizeof(usbPrivateData_t));
    if (!privateData) {
        free(instance);
        return NULL;
    }

#ifndef NO_USB
    privateData->usb_instance = new bsp::VirtualUSB();
    if (!privateData->usb_instance) {
        free(privateData);
        free(instance);
        return NULL;
    }

    // 初始化USB
    privateData->usb_instance->SetupTx(MAX_PRINT_LEN * 2);
    privateData->usb_instance->SetupRx(MAX_PRINT_LEN * 2);
#else
    free(privateData);
    free(instance);
    return NULL;
#endif

    privateData->usb_instance->RegisterCallback(callback, args);

    // 设置VTable和属性
    instance->vTable = &usbVTable;
    instance->baudRate = 115200;  // USB虚拟串口通常不需要设置波特率
    instance->mode = MODE_RXTX;
    instance->options = SERIAL_NOT_INVERTED;
    instance->privateData = privateData;

    return instance;
}

// 释放串口实例
void deleteSerialPort(serialPort_t* instance) {
    if (!instance)
        return;

    if (instance->vTable == &uartVTable) {
        uartPrivateData_t* privateData = (uartPrivateData_t*)instance->privateData;
        if (privateData) {
            if (privateData->uart_instance) {
                delete privateData->uart_instance;
            }
            free(privateData);
        }
    } else if (instance->vTable == &usbVTable) {
        usbPrivateData_t* privateData = (usbPrivateData_t*)instance->privateData;
        if (privateData) {
#ifndef NO_USB
            if (privateData->usb_instance) {
                delete privateData->usb_instance;
            }
#endif
            free(privateData);
        }
    }

    free(instance);
}
