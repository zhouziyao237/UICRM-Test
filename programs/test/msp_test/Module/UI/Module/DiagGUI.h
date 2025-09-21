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

#ifndef UICRM_DIAGGUI_H
#define UICRM_DIAGGUI_H

#include "OSD.h"
#include "StringGUI.h"

namespace RMFC {
    class DiagGUI {
      public:
        DiagGUI(UserInterface* UI, int16_t diag_X = 350, int16_t diag_Y = 850);
        ~DiagGUI();
        void Update(char* String, delay_t delay_function, int8_t color);
        void Clear(delay_t delay_func = [](uint32_t milli) { HAL_Delay(milli); });

      private:
        UserInterface* UI_;
        StringGUI* diag_string_[25];
        int16_t diag_X_;
        int16_t diag_Y_;
        int8_t count_;
    };
}  // namespace RMFC

#endif  // UICRM_DIAGGUI_H
