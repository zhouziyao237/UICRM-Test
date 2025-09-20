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
// Created by Administrator on 2025/9/19.
//

#pragma once

#include "Device/bsp_Serial/DeviceSerial.h"
#include "Module/RFS/RefereeSerial.h"
#include "arm_math.h"
#include "config/msg_data.h"
#include "utils.h"

namespace RMFC {

    typedef void (*delay_t)(uint32_t milli);

    /**
     * @brief 裁判系统UI类
     */
    /**
     * @brief Referee system UI class
     */
    class UserInterface {
      public:
        /**
         * @brief 构造函数
         * @param RefereeProtocol 裁判系统对象
         * @param uart 裁判系统对象
         */
        /**
         * @brief constructor
         * @param uart UART used by referee system
         * @param RefereeProtocol referee system object
         */
        UserInterface(serialPort_t* uart, RFS::RefereeProtocol* RefereeProtocol);
        /**
         * @brief 设置机器人ID
         * @param Robot_ID 需要设置的机器人ID
         * @return true为设置成功，false为设置失败
         */
        /**
         *
         * @param Robot_ID
         * @return
         */
        bool SetID(int Robot_ID);

        // 将某个图形的数据放入image

        static void LineDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                             uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                             uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);
        static void RectangleDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                                  uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                                  uint32_t start_x, uint32_t start_y, uint32_t end_x,
                                  uint32_t end_y);
        static void CircleDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                               uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                               uint32_t start_x, uint32_t start_y, uint32_t graph_radius);
        static void EllipseDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                                uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                                uint32_t start_x, uint32_t start_y, uint32_t x_length,
                                uint32_t y_length);
        static void ArcDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                            uint32_t graph_layer, uint32_t graph_color, uint32_t graph_startAngle,
                            uint32_t graph_endAngle, uint32_t graph_width, uint32_t start_x,
                            uint32_t start_y, uint32_t x_length, uint32_t y_length);
        static void FloatDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                              uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                              uint32_t graph_digit, uint32_t graph_width, uint32_t start_x,
                              uint32_t start_y, float graph_float);
        static void IntDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                            uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                            uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                            int graph_int);
        static void CharDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                             uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                             uint32_t char_length, uint32_t graph_width, uint32_t start_x,
                             uint32_t start_y);

        // 将data_buffer里面的数据作为 机器人交互数据->选手端UI绘图 发送出去
        size_t WriteData(uint8_t* data_buffer, content graph_content);

        int UIDelete(uint8_t del_operate, uint8_t del_layer);

        // 将若干个graphic_data_t复制到局部数组中，调用WriteData发送
        int GraphRefresh(int cnt, ...);

        // CharRefresh需要独立的一个 机器人交互数据 包
        int CharRefresh(graphic_data_t image, char* theString, int len);

        void DiagGUIInit(graphic_data_t* message, int len);
        void DiagGUIUpdate(int len);
        void DiagGUIClear(UserInterface* UI, RFS::RefereeProcessor* RefereeProcessor,
                          graphic_data_t* graph, int currCount);
        void AddMessage(graphic_data_t* graph, char* messageStr, int len);

      private:
        serialPort_t* uart_;
        RFS::RefereeProtocol* RefereeProtocol_;

        int Robot_ID_;
        int Client_ID_;
        graphic_data_t* gimbal_;
        graphic_data_t* chassis_;
        graphic_data_t* arrow_;
        graphic_data_t* crosshair_;
        graphic_data_t* bar_;
        graphic_data_t* percent_;
        graphic_data_t* diag_;
        graphic_data_t* cali_;
        int centerX_ = 960;
        int centerY_ = 540;
        int chassisX_ = 1300;
        int chassisY_ = 120;
        int gimbalLen_;
        int chassisLen_;
        int barStartX_ = 1500;
        int barStartY_ = 380;
        char percentStr_[30];
        int percentLen_;
        float cap_;
        int diagStartX_ = 1470;
        int diagStartY_ = 910;  // 420 - 910
        int messageCount_ = 0;
        int modeStartX_ = 1220;
        int modeStartY_ = 45;
    };
}  // namespace RMFC