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

#include "bsp_uart.h"
#include "connection_driver.h"
#include "dbus_package.h"

namespace remote {

    /**
     * @brief DBUS 遥控器接收类
     * @note 用于DJI DR16接收机
     */
    /**
     * @brief DBUS remote receiver class
     * @note used for DJI DR16 receiver
     */
    class DBUS : public bsp::UART, public driver::ConnectionDriver {
      public:
        /**
         * @brief 构造函数
         * @note 和uart类似，dbus需要时间进行初始化
         *
         * @param huart uart实例
         */
        /**
         * @brief intialize DBUS the same way as a generic UART peripheral
         * @note like uart, dbus needs time to initialize
         *
         * @param huart uart instance
         */
        DBUS(UART_HandleTypeDef* huart);

        // Add custom rx data handler
        void RxCompleteCallback() override final;

        // Return one decoded frame through the unified DBUS input API.
        RemoteInput GetInput() const;

        // rocker channel information
        /**
         * @note 遥控器的样式
         *
         * @note the style of the remote
         *
         * C4(
         * SWL*           *SWR
         *   C3-^       ^-C1
         * C2-<   >+ -<   >+C0
         *     +v       v+
         *
         */
        /**
         * @brief 获取右摇杆的x轴值
         */
        volatile int16_t ch0 = 0;
        /**
         * @brief 获取右摇杆的y轴值
         */
        volatile int16_t ch1 = 0;
        /**
         * @brief 获取左摇杆的x轴值
         */
        volatile int16_t ch2 = 0;
        /**
         * @brief 获取左摇杆的y轴值
         */
        volatile int16_t ch3 = 0;

        // left and right switch information
        /**
         * @brief 获取左拨杆的状态
         */
        volatile switch_t swl = MID;
        /**
         * @brief 获取右拨杆的状态
         */
        volatile switch_t swr = MID;
        // left track wheel
        /**
         * @brief 获取左波轮的x轴值
         */
        volatile int16_t ch4 = 0;
        // mouse movement and button information
        /**
         * @brief 获取鼠标的数据，仅限于DT7连接上电脑之后
         */
        mouse_t mouse{};
        // keyboard key information
        /**
         * @brief 获取键盘的数据，仅限于DT7连接上电脑之后
         */
        keyboard_t keyboard{};

        static const int16_t ROCKER_MIN = -RemoteInput::CHANNEL_MAX;
        static const int16_t ROCKER_MAX = RemoteInput::CHANNEL_MAX;

        uint32_t timestamp = 0U;

      private:
        RemoteInput input_frames_[2]{};
        volatile uint8_t active_input_index_ = 0U;
        volatile uint32_t input_generation_ = 0U;
    };

} /* namespace remote */
