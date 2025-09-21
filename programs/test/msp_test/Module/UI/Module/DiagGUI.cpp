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

#include "DiagGUI.h"

namespace RMFC {
    DiagGUI::DiagGUI(UserInterface* UI, int16_t diag_X, int16_t diag_Y) : UI_(UI) {
        diag_X_ = diag_X;
        diag_Y_ = diag_Y;
        count_ = 0;
        for (uint8_t i = 0; i < 25; i++) {
            diag_string_[i] = nullptr;
        }
    }

    DiagGUI::~DiagGUI() {
        for (uint8_t i = 0; i < 25; i++) {
            if (diag_string_[i] != nullptr) {
                diag_string_[i]->Delete();
                delete diag_string_[i];
                diag_string_[i] = nullptr;
            }
        }
    }

    void DiagGUI::Update(char* String, delay_t delay_function, int8_t color) {
        if (count_ >= 25) {
            return;
        }
        delay_function(100);
        if (color == -1)
            color = UI_Color_Main;
        char name[15];
        snprintf(name, 15, "M%d", count_);
        if (diag_string_[count_] == nullptr) {
            diag_string_[count_] =
                new StringGUI(UI_, String, diag_X_, diag_Y_ - count_ * 20, color, 10, name);
        }
        diag_string_[count_]->Init();
        count_++;
    }
    void DiagGUI::Clear(delay_t delay_func) {
        for (uint8_t i = 0; i < 25; i++) {
            delay_func(100);
            graphic_data_t temp;
            char name[15];
            snprintf(name, 15, "M%d", i);
            UI_->CharDraw(&temp, name, UI_Graph_Del, 4, UI_Color_Main, 10, 1, 2, diag_X_, diag_Y_);
            UI_->GraphRefresh(1, temp);

            delete diag_string_[i];
            diag_string_[i] = nullptr;
        }
        count_ = 0;
    }
}
