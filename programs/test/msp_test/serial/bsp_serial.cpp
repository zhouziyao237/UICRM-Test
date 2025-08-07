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

#include "bsp_serial.h"  // third party tiny-printf implemnetations

#include "bsp_uart.h"
#include "bsp_usb.h"
#include "cmsis_os.h"
#include "printf.h"

#define MAX_PRINT_LEN 1024
#define RX_SIGNAL (1 << 0)

#ifndef NO_USB
static bsp::VirtualUSB* Main_usb = NULL;
#endif

static char tx_buffer[MAX_PRINT_LEN];

extern int rx_lenght = 0;
extern char rx_buffer[MAX_PRINT_LEN];

void InitMainUart(UART_HandleTypeDef* huart, bool dma, uint32_t baudrate) {
    Main_uart = new bsp::UART(huart);
    if (baudrate != huart->Init.BaudRate)
        Main_uart->SetBaudrate(baudrate);
    Main_uart->SetupTx(MAX_PRINT_LEN * 2, dma);
    Main_uart->SetupRx(MAX_PRINT_LEN * 2, dma);
}

void InitMainUsb(void) {
#ifndef NO_USB
    Main_usb = new bsp::VirtualUSB();
#endif
}

uint32_t dump_write(const void* data, uint8_t length) {
    if (Main_uart)
        return Main_uart->Write((uint8_t*)data, length);
#ifndef NO_USB
    else if (Main_usb)
        return Main_usb->Write((uint8_t*)data, length);
#endif
    else
        return 0;
}

uint32_t dump_read(uint8_t** data) {
    if (Main_uart)
        return Main_uart->Read(data);
#ifndef NO_USB
    else if (Main_usb)
        return Main_usb->Read(data);
#endif
    else
        return 0;
}

uint32_t print(const char* format, ...) {
#ifdef NDEBUG
    UNUSED(format);
    UNUSED(print_buffer);
    return 0;
#else   // == #ifdef DEBUG
    va_list args;
    int length;

    va_start(args, format);
    length = vsnprintf(tx_buffer, MAX_PRINT_LEN, format, args);
    va_end(args);

    return dump_write(tx_buffer, length);
#endif  // #ifdef NDEBUG
}

uint32_t read(uint8_t** data) {
    return dump_read(data);
}

void ReadUARTTask(void* arg) {
    UNUSED(arg);
    uint8_t* data;

    while (true) {
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {
            rx_lenght = Main_uart->Read(&data);
            if (rx_lenght > 0 && rx_lenght <= MAX_PRINT_LEN) {
                memcpy(rx_buffer, data, rx_lenght);
            }
        }
    }
}

#ifndef NO_USB
void ReadUSBTask(void* arg) {
    UNUSED(arg);
    uint8_t* data;

    while (true) {
        uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);
        if (flags & RX_SIGNAL) {
            rx_lenght = Main_usb->Read(&data);
            if (rx_lenght > 0 && rx_lenght <= MAX_PRINT_LEN) {
                memcpy(rx_buffer, data, rx_lenght);
            }
        }
    }
}
#endif
