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

#ifndef UICRM_STRINGGUI_H
#define UICRM_STRINGGUI_H

#include "OSD.h"

namespace RMFC {
    class StringGUI {
      public:
        StringGUI(UserInterface* UI, char* string_content, int16_t string_X = 1500,
                  int16_t string_Y = 380, int8_t color = UI_Color_Main, int16_t string_size = 15,
                  char* string_name = nullptr);
        ~StringGUI();
        graphic_data_t InitBulk();
        void Init();
        void InitString(char* string = nullptr);
        graphic_data_t DeleteBulk();
        void Delete();
        void Update(char* string, int8_t color = -1, int16_t size = -1);
        static uint8_t string_count_;

      private:
        UserInterface* UI_;
        graphic_data_t string_;
        char string_name_[15];
        char* string_content_;
        uint8_t string_length_;
        int16_t string_X_;
        int16_t string_Y_;
        int16_t string_size_;
        uint8_t string_ID_;
        int8_t color_;
    };
}  // namespace RMFC

#endif  // UICRM_STRINGGUI_H
