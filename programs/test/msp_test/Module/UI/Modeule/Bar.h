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

#ifndef UICRM_BAR_H
#define UICRM_BAR_H

#include "OSD.h"

namespace RMFC {
    class Bar {
    public:
        Bar(int16_t barStartX = 1500, int16_t barStartY = 350, int16_t barWidth = 200,
            int16_t barHeight = 50, uint8_t color = UI_Color_Orange,
            uint8_t frame_color = UI_Color_Pink, bool isVertical = false);
        graphic_data_t Init();
        graphic_data_t Delete();
        graphic_data_t InitFrame();
        graphic_data_t DeleteFrame();
        graphic_data_t Update(float percent, int8_t color = -1);
        static uint8_t barcount_;

    private:
        uint8_t barID_;
        int16_t barStartX_;
        int16_t barStartY_;
        int16_t barWidth_;
        int16_t barHeight_;
        bool isVertical_;
        float percent_;
        uint8_t color_;
        uint8_t frame_color_;
        graphic_data_t barFrame_;
        graphic_data_t bar_;
        char name_[15];
        char name_frame_[15];
    };
}

#endif  // UICRM_BAR_H
