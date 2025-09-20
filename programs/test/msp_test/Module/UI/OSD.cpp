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

#include "OSD.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "config/msg_data.h"
#include "utils.h"

namespace RMFC {
    UserInterface::UserInterface(serialPort_t* uart, RFS::RefereeProtocol* RefereeProtocol)
        : uart_(uart), RefereeProtocol_(RefereeProtocol) {
    }

    bool UserInterface::SetID(int Robot_ID) {
        switch (Robot_ID) {
            case 1:
                Robot_ID_ = UI_Data_RobotID_RHero;
                Client_ID_ = UI_Data_CilentID_RHero;
                break;
            case 2:
                Robot_ID_ = UI_Data_RobotID_REngineer;
                Client_ID_ = UI_Data_CilentID_REngineer;
                break;
            case 3:
                Robot_ID_ = UI_Data_RobotID_RStandard1;
                Client_ID_ = UI_Data_CilentID_RStandard1;
                break;
            case 4:
                Robot_ID_ = UI_Data_RobotID_RStandard2;
                Client_ID_ = UI_Data_CilentID_RStandard2;
                break;
            case 5:
                Robot_ID_ = UI_Data_RobotID_RStandard3;
                Client_ID_ = UI_Data_CilentID_RStandard3;
                break;
            case 6:
                Robot_ID_ = UI_Data_RobotID_RAerial;
                Client_ID_ = UI_Data_CilentID_RAerial;
                break;
            case 101:
                Robot_ID_ = UI_Data_RobotID_BHero;
                Client_ID_ = UI_Data_CilentID_BHero;
                break;
            case 102:
                Robot_ID_ = UI_Data_RobotID_BEngineer;
                Client_ID_ = UI_Data_CilentID_BEngineer;
                break;
            case 103:
                Robot_ID_ = UI_Data_RobotID_BStandard1;
                Client_ID_ = UI_Data_CilentID_BStandard1;
                break;
            case 104:
                Robot_ID_ = UI_Data_RobotID_BStandard2;
                Client_ID_ = UI_Data_CilentID_BStandard2;
                break;
            case 105:
                Robot_ID_ = UI_Data_RobotID_BStandard3;
                Client_ID_ = UI_Data_CilentID_BStandard3;
                break;
            case 106:
                Robot_ID_ = UI_Data_RobotID_BAerial;
                Client_ID_ = UI_Data_CilentID_BAerial;
                break;
            default:
                return false;
        }
        return true;
    }

    /**
     *
     * @param image pointer to a graphic_data_t variable; stores image data
     * @param name image name
     * @param graphOperate graph operations; check document
     * @param graph_layer layer 0 to 9
     * @param graph_color graph color
     * @param graph_width graph width
     * @param start_x starting x coordinate
     * @param start_y starting y coordinate
     * @param end_x ending x coordinate
     * @param end_y ending y coordinate
     */
    void UserInterface::LineDraw(graphic_data_t* image, const char name[3], uint32_t graphOperate,
                                 uint32_t graph_layer, uint32_t graph_color, uint32_t graph_width,
                                 uint32_t start_x, uint32_t start_y, uint32_t end_x,
                                 uint32_t end_y) {
        for (int i = 0; i < 3 && name[i] != 0; i++)
            image->graphic_name[2 - i] = name[i];
        image->operate_type = graphOperate;
        image->graphic_type = UI_Graph_Line;
        image->layer = graph_layer;
        image->color = graph_color;
        image->width = graph_width;
        image->start_x = start_x;
        image->start_y = start_y;
        image->end_x = end_x;
        image->end_y = end_y;
    }

    /**
     *
     * @param image pointer to a graphic_data_t variable; stores image data
     * @param name image name
     * @param graph_operate graph operations; check document
     * @param graph_layer layer 0 to 9
     * @param graph_color graph color
     * @param graph_width graph width
     * @param start_x starting x coordinate
     * @param start_y starting y coordinate
     * @param end_x ending x coordinate
     * @param end_y ending y coordinate
     */
    void UserInterface::RectangleDraw(graphic_data_t* image, const char name[3],
                                      uint32_t graph_operate, uint32_t graph_layer,
                                      uint32_t graph_color, uint32_t graph_width, uint32_t start_x,
                                      uint32_t start_y, uint32_t end_x, uint32_t end_y) {
        for (int i = 0; i < 3 && name[i] != 0; i++)
            image->graphic_name[2 - i] = name[i];
        image->operate_type = graph_operate;
        image->graphic_type = UI_Graph_Rectangle;
        image->layer = graph_layer;
        image->color = graph_color;
        image->width = graph_width;
        image->start_x = start_x;
        image->start_y = start_y;
        image->end_x = end_x;
        image->end_y = end_y;
    }

    /**
     *
     * @param image pointer to a graphic_data_t variable; stores image data
     * @param name image name
     * @param graph_operate graph operations; check document
     * @param graph_layer layer 0 to 9
     * @param graph_color graph color
     * @param graph_width graph width
     * @param start_x starting x coordinate
     * @param start_y starting y coordinate
     * @param graph_radius graph radius
     */
    void UserInterface::CircleDraw(graphic_data_t* image, const char name[3],
                                   uint32_t graph_operate, uint32_t graph_layer,
                                   uint32_t graph_color, uint32_t graph_width, uint32_t start_x,
                                   uint32_t start_y, uint32_t graph_radius) {
        for (int i = 0; i < 3 && name[i] != 0; i++)
            image->graphic_name[2 - i] = name[i];
        image->operate_type = graph_operate;
        image->graphic_type = UI_Graph_Circle;
        image->layer = graph_layer;
        image->color = graph_color;
        image->width = graph_width;
        image->start_x = start_x;
        image->start_y = start_y;
        image->radius = graph_radius;
    }

    /**
     *
     * @param image pointer to a graphic_data_t variable; stores image data
     * @param name image name
     * @param graph_operate graph operations; check document
     * @param graph_layer layer 0 to 9
     * @param graph_color graph color
     * @param graph_width graph width
     * @param start_x starting (center) x coordinate
     * @param start_y starting (center) y coordinate
     * @param x_length horizontal semi-axis length
     * @param y_length vertical semi-axis length
     */
    void UserInterface::EllipseDraw(graphic_data_t* image, const char name[3],
                                    uint32_t graph_operate, uint32_t graph_layer,
                                    uint32_t graph_color, uint32_t graph_width, uint32_t start_x,
                                    uint32_t start_y, uint32_t x_length, uint32_t y_length) {
        for (int i = 0; i < 3 && name[i] != 0; i++)
            image->graphic_name[2 - i] = name[i];
        image->operate_type = graph_operate;
        image->graphic_type = UI_Graph_Ellipse;
        image->layer = graph_layer;
        image->color = graph_color;
        image->width = graph_width;
        image->start_x = start_x;
        image->start_y = start_y;
        image->end_x = x_length;
        image->end_y = y_length;
    }

    /**
     *
     * @param image pointer to a graphic_data_t variable; stores image data
     * @param name image name
     * @param graph_operate graph operations; check document
     * @param graph_layer layer 0 to 9
     * @param graph_startAngle starting angle
     * @param graph_endAngle ending angle
     * @param graph_width graph width
     * @param start_x starting (center) x coordinate
     * @param start_y starting (center) y coordinate
     * @param x_length horizontal semi-axis length
     * @param y_length vertical semi-axis length
     */
    void UserInterface::ArcDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                                uint32_t graph_layer, uint32_t graph_color,
                                uint32_t graph_startAngle, uint32_t graph_endAngle,
                                uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                                uint32_t x_length, uint32_t y_length) {
        for (int i = 0; i < 3 && name[i] != 0; i++)
            image->graphic_name[2 - i] = name[i];
        image->operate_type = graph_operate;
        image->graphic_type = UI_Graph_Arc;
        image->layer = graph_layer;
        image->color = graph_color;
        image->start_angle = graph_startAngle;
        image->end_angle = graph_endAngle;
        image->width = graph_width;
        image->start_x = start_x;
        image->start_y = start_y;
        image->end_x = x_length;
        image->end_y = y_length;
    }

    /**
     *
     * @param image pointer to a graphic_data_t variable; stores image data
     * @param name image name
     * @param graph_operate graph operations; check document
     * @param graph_layer layer 0 to 9
     * @param graph_color graph color
     * @param graph_size graph size
     * @param graph_digit number of digits after decimal
     * @param graph_width graph width
     * @param start_x starting x coordinate
     * @param start_y starting y coordinate
     * @param graph_float float to be displayed
     */
    void UserInterface::FloatDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                                  uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                                  uint32_t graph_digit, uint32_t graph_width, uint32_t start_x,
                                  uint32_t start_y, float graph_float) {
        for (int i = 0; i < 3 && name[i] != 0; i++)
            image->graphic_name[2 - i] = name[i];
        image->operate_type = graph_operate;
        image->graphic_type = UI_Graph_Float;
        image->layer = graph_layer;
        image->color = graph_color;
        image->start_angle = graph_size;
        image->end_angle = graph_digit;
        image->width = graph_width;
        image->start_x = start_x;
        image->start_y = start_y;
        float float_data = graph_float;
        int32_t float2int_data;
        memcpy(&float2int_data, (uint8_t*)&float_data, 4);
        image->radius = float2int_data & 0x3FF;
        image->end_x = (float2int_data & 0x1FFC00) >> 10;
        image->end_y = (float2int_data & 0xFFE00000) >> 21;
    }

    void UserInterface::IntDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                                uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                                uint32_t graph_width, uint32_t start_x, uint32_t start_y,
                                int graph_int) {
        for (int i = 0; i < 3 && name[i] != 0; i++)
            image->graphic_name[2 - i] = name[i];
        image->operate_type = graph_operate;
        image->graphic_type = UI_Graph_Int;
        image->layer = graph_layer;
        image->color = graph_color;
        image->start_angle = graph_size;
        image->width = graph_width;
        image->start_x = start_x;
        image->start_y = start_y;
        int32_t int_data = graph_int;
        image->radius = int_data & 0x3FF;
        image->end_x = (int_data & 0x1FFC00) >> 10;
        image->end_y = (int_data & 0xFFE00000) >> 21;
    }

    /**
     *
     * @param image pointer to a graphic_data_t variable; stores image data
     * @param name image name
     * @param graph_operate graph operations; check document
     * @param graph_layer layer 0 to 9
     * @param graph_color graph color
     * @param graph_size graph size
     * @param char_length character length
     * @param graph_width graph width
     * @param start_x starting x coordinate
     * @param start_y starting y coordinate
     */
    void UserInterface::CharDraw(graphic_data_t* image, const char name[3], uint32_t graph_operate,
                                 uint32_t graph_layer, uint32_t graph_color, uint32_t graph_size,
                                 uint32_t char_length, uint32_t graph_width, uint32_t start_x,
                                 uint32_t start_y) {
        for (int i = 0; i < 3 && name[i] != 0; i++)
            image->graphic_name[2 - i] = name[i];
        image->operate_type = graph_operate;
        image->graphic_type = UI_Graph_Char;
        image->layer = graph_layer;
        image->color = graph_color;
        image->start_angle = graph_size;
        image->end_angle = char_length;
        image->width = graph_width;
        image->start_x = start_x;
        image->start_y = start_y;
    }

    size_t UserInterface::WriteData(uint8_t* data_buffer, content graph_content) {
        switch (graph_content) {
            case SINGLE_GRAPH:
                memcpy((uint8_t*)(&RefereeProtocol_->Refereedata_->graphic_single), data_buffer,
                       sizeof(graphic_single_t));
                break;
            case DOUBLE_GRAPH:
                memcpy((uint8_t*)(&RefereeProtocol_->Refereedata_->graphic_double), data_buffer,
                       sizeof(graphic_double_t));
                break;
            case FIVE_GRAPH:
                memcpy((uint8_t*)(&RefereeProtocol_->Refereedata_->graphic_five), data_buffer,
                       sizeof(graphic_five_t));
                break;
            case SEVEN_GRAPH:
                memcpy((uint8_t*)(&RefereeProtocol_->Refereedata_->graphic_seven), data_buffer,
                       sizeof(graphic_seven_t));
                break;
            case CHAR_GRAPH:
                memcpy((uint8_t*)(&RefereeProtocol_->Refereedata_->graphic_character), data_buffer,
                       sizeof(graphic_character_t));
                break;
            case DELETE_GRAPH:
                memcpy((uint8_t*)(&RefereeProtocol_->Refereedata_->graphic_delete), data_buffer,
                       sizeof(graphic_delete_t));
                break;
            default:
                break;
        }
        RefereeProtocol_->PrepareUIContent(graph_content);
        size_t length = RefereeProtocol_->Transmit(STUDENT_INTERACTIVE);
        return length;
    }

    int UserInterface::UIDelete(uint8_t del_operate, uint8_t del_layer) {
        graphic_delete_t del;
        del.header.data_cmd_id = UI_Data_ID_Del;
        del.header.sender_ID = Robot_ID_;
        del.header.receiver_ID = Client_ID_;
        del.operate_type = del_operate;
        del.layer = del_layer;
        return WriteData(reinterpret_cast<uint8_t*>(&del), DELETE_GRAPH);
        //        int length = sizeof(graphic_delete_t);
        //        memcpy(data_buffer, &del, length);
        //        return length;
    }

    int UserInterface::GraphRefresh(int cnt, ...) {
        va_list arg;
        va_start(arg, cnt);
        robot_interactive_data_header_t header;
        header.sender_ID = Robot_ID_;
        header.receiver_ID = Client_ID_;
        //        int length = -1;
        content graph_content;
        switch (cnt) {
            case 1: {
                header.data_cmd_id = UI_Data_ID_Draw1;
                graphic_single_t graph;
                graph.header = header;
                graph.graphic_data_struct = va_arg(arg, graphic_data_t);
                //                length = sizeof(graphic_single_t);
                graph_content = SINGLE_GRAPH;
                return WriteData((uint8_t*)(&graph), graph_content);
                //                memcpy(data_buffer, &graph, length);
                break;
            }
            case 2: {
                header.data_cmd_id = UI_Data_ID_Draw2;
                graphic_double_t graph;
                graph.header = header;
                for (int i = 0; i < cnt; ++i)
                    graph.graphic_data_struct[i] = va_arg(arg, graphic_data_t);
                //                length = sizeof(graphic_double_t);
                graph_content = DOUBLE_GRAPH;
                return WriteData((uint8_t*)(&graph), graph_content);
                //                memcpy(data_buffer, &graph, length);
                break;
            }
            case 5: {
                header.data_cmd_id = UI_Data_ID_Draw5;
                graphic_five_t graph;
                graph.header = header;
                for (int i = 0; i < cnt; ++i)
                    graph.graphic_data_struct[i] = va_arg(arg, graphic_data_t);
                //                length = sizeof(graphic_five_t);
                graph_content = FIVE_GRAPH;
                return WriteData((uint8_t*)(&graph), graph_content);
                //                memcpy(data_buffer, &graph, length);
                break;
            }
            case 7: {
                header.data_cmd_id = UI_Data_ID_Draw7;
                graphic_seven_t graph;
                graph.header = header;
                for (int i = 0; i < cnt; ++i)
                    graph.graphic_data_struct[i] = va_arg(arg, graphic_data_t);
                //                length = sizeof(graphic_seven_t);
                graph_content = SEVEN_GRAPH;
                return WriteData((uint8_t*)(&graph), graph_content);
                //                memcpy(data_buffer, &graph, length);
                break;
            }
            default:;
        }
        return -1;

        //        return length;
    }

    int UserInterface::CharRefresh(graphic_data_t image, char* theString, int len) {
        graphic_character_t char_graph;
        char_graph.header.data_cmd_id = UI_Data_ID_DrawChar;
        char_graph.header.sender_ID = Robot_ID_;
        char_graph.header.receiver_ID = Client_ID_;
        char_graph.graphic_data_struct = image;
        if (len > 30 || len <= 0)
            return -1;
        memset(char_graph.data, 0, sizeof(char_graph.data));
        memcpy(char_graph.data, theString, len);
        return WriteData((uint8_t*)(&char_graph), CHAR_GRAPH);
        //        int length = sizeof(graphic_character_t);
        //        memcpy(data_buffer, &char_graph, length);
        //        return length;
    }

    void UserInterface::DiagGUIInit(graphic_data_t* message, int len) {
        diag_ = message;
        CharDraw(message, "M0", UI_Graph_Add, 2, UI_Color_Pink, 10, len, 2, diagStartX_,
                 diagStartY_);
    }

    void UserInterface::DiagGUIUpdate(int len) {
        int currY = diagStartY_ - messageCount_ * 20;
        char name[15];
        snprintf(name, 15, "M%d", messageCount_);
        CharDraw(diag_, name, UI_Graph_Add, 2, UI_Color_Pink, 10, len, 2, diagStartX_, currY);
    }

    void UserInterface::AddMessage(graphic_data_t* graph, char* messageStr, int len) {
        messageCount_++;
        if (messageCount_ > 25)
            return;
        DiagGUIUpdate(len);
        CharRefresh(*graph, messageStr, len);
    }

    void UserInterface::DiagGUIClear(UserInterface* UI, RFS::RefereeProcessor* RefereeProcessor,
                                     graphic_data_t* graph, int currCount) {
        char str[] = " ";
        char name[15];
        snprintf(name, 15, "M%d", currCount);
        UI->CharDraw(diag_, name, UI_Graph_Change, 2, UI_Color_Pink, 10, 30, 2, diagStartX_,
                     diagStartY_ - currCount * 20);
        UI->CharRefresh(*graph, str, 1);
        RefereeProcessor->PrepareUIContent(CHAR_GRAPH);
    }
}  // namespace RMFC