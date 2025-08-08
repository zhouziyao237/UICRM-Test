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

#include <cinttypes>

#include "bsp_uart.h"
#include "bsp_usb.h"
#include "usart.h"

#define MAX_PRINT_LEN 1024
#define RX_SIGNAL (1 << 0)

class Serial {
  public:
    /**
     * @brief 初始化主UART接口
     * @param huart UART句柄指针
     * @param dma 是否使用DMA传输
     * @param baudrate 波特率设置
     */
    void InitMainUart(UART_HandleTypeDef* huart, bool dma, uint32_t baudrate);

#ifndef NO_USB
    /**
     * @brief 初始化主USB接口
     */
    void InitMainUsb(void);
#endif

    /**
     * @brief 底层数据发送函数
     * @param data 要发送的数据指针
     * @param length 数据长度
     * @return 实际发送的字节数
     */
    uint32_t dump_write(const void* data, uint8_t length);

    /**
     * @brief 底层数据读取函数
     * @param data 指向接收数据缓冲区的指针
     * @return 实际读取的字节数
     */
    uint32_t dump_read(uint8_t** data);

    /**
     * @brief 格式化打印输出函数
     * @param format 格式化字符串
     * @param ... 可变参数列表
     * @return 输出的字符数
     */
    uint32_t print(const char* format, ...);

    /**
     * @brief 读取数据函数
     * @param data 指向接收数据缓冲区的指针
     * @return 实际读取的字节数
     */
    uint32_t read(uint8_t** data);

    /**
     * @brief UART接收数据处理任务
     * @param arg 任务参数
     */
    void ReadUARTTask(void* arg);

#ifndef NO_USB
    /**
     * @brief USB接收数据处理任务
     * @param arg 任务参数
     */
    void ReadUSBTask(void* arg);
#endif

  private:

    bsp::UART* Main_uart = nullptr;
#ifndef NO_USB
    bsp::VirtualUSB* Main_usb = nullptr;
#endif
    int rx_lenght = 0;
    char tx_buffer[MAX_PRINT_LEN];
    char rx_buffer[MAX_PRINT_LEN];
};