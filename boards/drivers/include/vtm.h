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

#include <cstdint>
#include <cstring>

#include "bsp_uart.h"
#include "connection_driver.h"
#include "vtm_package.h"
#include "bsp_thread.h"

namespace remote {

    /* rocker range and deadzones */

    // 通信参数宏定义
    #define BAUD_RATE 921600                    // 波特率
    #define TIME_INTERVAL_MS 14                 // 时间间隔(毫秒)
    #define PACKET_SIZE 21                      // 单包字节数
    #define BUFFER_SIZE 100

    // VTM相关常量
    #define VTM_FRAME_HEADER_1 0xA9             // 帧头第一字节
    #define VTM_FRAME_HEADER_2 0x53             // 帧头第二字节
    #define VTM_CRC_START_POS 0                 // CRC计算起始位置
    #define VTM_CRC_LENGTH 19                   // CRC计算数据长度
    #define VTM_FRAME_TOTAL_LENGTH 21           // 完整帧长度

    #define CRC16_INIT 0xFFFF

    #define VTM_ROCKER_MID 1024
    #define VTM_ROCKER_MIN 364
    #define VTM_ROCKER_MAX 1684
    #define VTM_ROCKER_ZERO_DRIFT 10  // Range of possible drift around initial position

    #define MAKE_IDLE_TIM TIME_INTERVAL_MS

    typedef struct {
        uint8_t* data;
        int length;
    } package_t;

    // VTM专用数据类型
    typedef enum {
        VTM_GEAR_C = 0,  // 低速档
        VTM_GEAR_N = 1,  // 中速档
        VTM_GEAR_S = 2,  // 高速档
    } vtm_gear_t;

    // helper struct for decoding raw bytes - 直接使用现有结构体
    typedef struct {
        /* 帧头 */
        uint8_t header1;
        uint8_t header2;

        /* 摇杆通道 */
        uint64_t ch0 : 11;
        uint64_t ch1 : 11;
        uint64_t ch2 : 11;
        uint64_t ch3 : 11;

        /* 开关和按键 */
        uint64_t gear_switch : 2;
        uint64_t pause_button : 1;
        uint64_t custom_left : 1;
        uint64_t custom_right : 1;

        /* 额外通道 */
        uint64_t ch4 : 11;
        uint64_t ch5 : 1;

        /* 鼠标数据 - 直接嵌入现有结构体 */
        vtm_mouse_t mouse;

        /* 键盘数据 - 直接嵌入现有结构体 */
        vtm_keyboard_t keyboard;

        /* CRC校验 */
        uint16_t crc16;
    } __attribute__((packed)) vtm_raw_t;

    /**
     * @brief VTM 图传接收类
     * @note 用于RoboMaster VTM图传模块接收端
     */
    class VTM : public driver::ConnectionDriver {
    public:
        /**
         * @brief 构造函数
         * @note 和uart类似，vtm需要时间进行初始化
         *
         * @param huart uart实例
         */
        explicit VTM(bsp::UART* uart);

        bool IsReady();

        void VTMupdata(uint8_t* data);

        bool vtm_start_flag = false;

        // rocker channel information
        /**
         * @note 遥控器的样式
         *
         * 右摇杆水平方向 (ch0)
         * 右摇杆垂直方向 (ch1)
         * 左摇杆垂直方向 (ch2)
         * 左摇杆水平方向 (ch3)
         * 拨轮 (ch4)
         * 扳机键 (ch5)
         */
        /**
         * @brief 获取右摇杆的x轴值
         */
        volatile int16_t ch0;
        /**
         * @brief 获取右摇杆的y轴值
         */
        volatile int16_t ch1;
        /**
         * @brief 获取左摇杆的y轴值
         */
        volatile int16_t ch2;
        /**
         * @brief 获取左摇杆的x轴值
         */
        volatile int16_t ch3;
        /**
         * @brief 获取拨轮的值
         */
        volatile int16_t ch4;
        /**
         * @brief 获取扳机键的值
         */
        volatile int16_t ch5;

        // switch and button information
        /**
         * @brief 获取挡位切换开关的状态
         */
        volatile vtm_gear_t gear_switch;
        /**
         * @brief 获取暂停按键的状态
         */
        volatile bool pause_button;
        /**
         * @brief 获取自定义左按键的状态
         */
        volatile bool custom_left;
        /**
         * @brief 获取自定义右按键的状态
         */
        volatile bool custom_right;

        // mouse movement and button information
        /**
         * @brief 获取鼠标的数据
         */
        vtm_mouse_t mouse;
        // keyboard key information
        /**
         * @brief 获取键盘的数据
         */
        vtm_keyboard_t keyboard;

        static const int16_t ROCKER_MIN = -660;
        static const int16_t ROCKER_MAX = 660;

        uint32_t timestamp;

    private:
        bsp::UART* uart_ = nullptr;

        uint8_t* read_ptr_ = nullptr;
        uint32_t read_len_ = 0;
        uint8_t bufferRx[BUFFER_SIZE] = {0};

        static void RxCallback(void* args);

    };

} /* namespace remote */
