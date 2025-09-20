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

#ifndef UICRM_CAPGUI_H
#define UICRM_CAPGUI_H

#include "OSD.h"
#include "Bar.h"

namespace RMFC {

    class CapGUI {
      public:
        CapGUI(UserInterface* UI, char* Cap_name, int16_t Cap_bar_X = 1500, int16_t Cap_bar_Y = 380,
               int16_t Cap_bar_width = 310, int16_t Cap_bar_height = 20);
        ~CapGUI();
        void Init();
        void InitName();
        void Delete();
        void DeleteName();
        void UpdateBulk(float percent, graphic_data_t* bar = nullptr,
                        graphic_data_t* cap_percent = nullptr);
        void Update(float Percent);

      private:
        UserInterface* UI_;
        Bar* cap_bar_;
        graphic_data_t barFrame_;
        graphic_data_t bar_;
        graphic_data_t cap_percent_;
        char cap_percent_name_[15];
        graphic_data_t cap_name_;
        char cap_name_name_[15];
        graphic_data_t empty_;
        char empty_name_[15];
        char* cap_name_str_;
        delay_t delay_function_;
        int16_t cap_bar_X_;
        int16_t cap_bar_Y_;
        int16_t cap_bar_height_;
        int16_t cap_bar_width_;
        int8_t cap_ID_;
        static uint8_t cap_count_;
        int8_t name_length_;
    };
}  // namespace RMFC

#endif  // UICRM_CAPGUI_H
