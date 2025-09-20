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
// Created by Administrator on 2025/9/20.
//

#include "StringGUI.h"

namespace RMFC {
    uint8_t StringGUI::string_count_ = 0;

    StringGUI::StringGUI(UserInterface* UI, char* string_content, int16_t string_X,
                         int16_t string_Y, int8_t color, int16_t string_size, char* string_name)
        : UI_(UI), string_content_(string_content) {
        string_X_ = string_X;
        string_Y_ = string_Y;
        string_size_ = string_size;
        color_ = color;
        string_length_ = strlen(string_content_);
        if (string_name == nullptr) {
            string_ID_ = StringGUI::string_count_;
            StringGUI::string_count_++;
            memset(string_name_, ' ', 15);
            snprintf(string_name_, 15, "S%d", string_ID_);
        } else {
            strcpy(string_name_, string_name);
        }
    }

    StringGUI::~StringGUI() {
    }

    graphic_data_t StringGUI::InitBulk() {
        UI_->CharDraw(&string_, string_name_, UI_Graph_Add, 4, color_, string_size_, string_length_,
                      2, string_X_, string_Y_);
        return string_;
    }

    void StringGUI::Init() {
        InitBulk();
        UI_->CharRefresh(string_, string_content_, string_length_);
    }

    void StringGUI::InitString(char* string) {
        if (string != nullptr)
            string_content_ = string;
        string_length_ = strlen(string_content_);
        UI_->CharDraw(&string_, string_name_, UI_Graph_Change, 4, color_, string_size_,
                      string_length_, 2, string_X_, string_Y_);
        UI_->CharRefresh(string_, string_content_, string_length_);
    }

    graphic_data_t StringGUI::DeleteBulk() {
        UI_->CharDraw(&string_, string_name_, UI_Graph_Del, 4, color_, string_size_, string_length_,
                      2, string_X_, string_Y_);
        return string_;
    }

    void StringGUI::Delete() {
        DeleteBulk();
        UI_->GraphRefresh(1, string_);
    }

    void StringGUI::Update(char* string, int8_t color, int16_t size) {
        if (string != nullptr)
            string_content_ = string;
        if (color != -1)
            color_ = color;
        if (size != -1)
            string_size_ = size;
        string_length_ = strlen(string_content_);
        UI_->CharDraw(&string_, string_name_, UI_Graph_Change, 4, color_, string_size_,
                      string_length_, 2, string_X_, string_Y_);
        UI_->CharRefresh(string_, string_content_, string_length_);
    }
}  // namespace RMFC