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

#include "CapGUI.h"

#include "Bar.h"

namespace RMFC {
    uint8_t CapGUI::cap_count_ = 0;

    CapGUI::CapGUI(UserInterface* UI, char* cap_name, int16_t cap_bar_X, int16_t cap_bar_Y,
                   int16_t cap_bar_width, int16_t cap_bar_height)
        : UI_(UI), cap_name_str_(cap_name) {
        cap_bar_X_ = cap_bar_X;
        cap_bar_Y_ = cap_bar_Y;
        cap_bar_height_ = cap_bar_height;
        cap_bar_width_ = cap_bar_width;
        cap_bar_ = new Bar(cap_bar_X_, cap_bar_Y_, cap_bar_width_, cap_bar_height_, UI_Color_Green,
                           UI_Color_Yellow);
        cap_ID_ = CapGUI::cap_count_;
        CapGUI::cap_count_++;
        Init();
    }

    void CapGUI::Init() {
        name_length_ = strlen(cap_name_str_);
        bar_ = cap_bar_->Init();
        barFrame_ = cap_bar_->InitFrame();
        memset(cap_name_name_, ' ', 15);
        memset(empty_name_, ' ', 15);
        memset(cap_percent_name_, ' ', 15);
        snprintf(cap_name_name_, 15, "V%d", cap_ID_);
        snprintf(cap_percent_name_, 15, "v%d", cap_ID_);
        snprintf(empty_name_, 15, "W%d", cap_ID_);
        UI_->CharDraw(&cap_name_, cap_name_name_, UI_Graph_Add, 1, UI_Color_Yellow, 15,
                      name_length_ + 5, 2, cap_bar_X_, cap_bar_Y_ - 10);
        UI_->IntDraw(&cap_percent_, cap_percent_name_, UI_Graph_Add, 3, UI_Color_Yellow, 15, 2,
                     cap_bar_X_ + (name_length_ + 1) * 15, cap_bar_Y_ - 10, 100);

        UI_->CircleDraw(&empty_, empty_name_, UI_Graph_Add, 0, UI_Color_Green, 0, 0, 0, 0);
        UI_->GraphRefresh(5, barFrame_, bar_, cap_percent_, cap_name_, empty_);
    }
    void CapGUI::Delete() {
        bar_ = cap_bar_->Delete();
        barFrame_ = cap_bar_->DeleteFrame();
        UI_->CharDraw(&cap_name_, cap_name_name_, UI_Graph_Del, 1, UI_Color_Yellow, 15,
                      name_length_ + 5, 2, cap_bar_X_, cap_bar_Y_ - 10);
        UI_->IntDraw(&cap_percent_, cap_percent_name_, UI_Graph_Del, 3, UI_Color_Yellow, 15, 2,
                     cap_bar_X_ + (name_length_ + 1) * 15, cap_bar_Y_ - 10, 100);
        UI_->CircleDraw(&empty_, empty_name_, UI_Graph_Del, 0, UI_Color_Green, 0, 0, 0, 0);
        UI_->GraphRefresh(5, barFrame_, bar_, cap_name_, cap_percent_, empty_);
    }
    void CapGUI::InitName() {
        char theRealName[15];
        snprintf(theRealName, 15, "%s     %%", cap_name_str_);
        UI_->CharDraw(&cap_name_, cap_name_name_, UI_Graph_Change, 1, UI_Color_Yellow, 15,
                      name_length_ + 5, 2, cap_bar_X_, cap_bar_Y_ - 10);
        UI_->CharRefresh(cap_name_, theRealName, strlen(theRealName));
    }
    void CapGUI::DeleteName() {
    }
    void CapGUI::UpdateBulk(float percent, graphic_data_t* bar, graphic_data_t* cap_percent) {
        percent = clip<float>(percent, 0, 1);
        if (percent > 0.8)
            bar_ = cap_bar_->Update(percent, UI_Color_Green);
        else if (percent > 0.2)
            bar_ = cap_bar_->Update(percent, UI_Color_Orange);
        else
            bar_ = cap_bar_->Update(percent, UI_Color_Pink);
        UI_->IntDraw(&cap_percent_, cap_percent_name_, UI_Graph_Change, 3, UI_Color_Yellow, 15, 2,
                     cap_bar_X_ + (name_length_ + 1) * 15, cap_bar_Y_ - 10,
                     (int32_t)(100 * percent));
        if (bar != nullptr)
            *bar = bar_;
        if (cap_percent != nullptr)
            *cap_percent = cap_percent_;
    }
    void CapGUI::Update(float percent) {
        UpdateBulk(percent);
        UI_->GraphRefresh(2, bar_, cap_percent_);
    }
}  // namespace RMFC
