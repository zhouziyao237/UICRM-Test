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

#include "Bar.h"

#include "OSD.h"

namespace RMFC {

    uint8_t Bar::barcount_ = 0;

    Bar::Bar(int16_t barStartX, int16_t barStartY, int16_t barWidth, int16_t barHeight,
             uint8_t color, uint8_t frame_color, bool isVertical) {
        barStartX_ = barStartX;
        barStartY_ = barStartY;
        barWidth_ = barWidth;
        barHeight_ = barHeight;
        isVertical_ = isVertical;
        color_ = color;
        frame_color_ = frame_color;
        barID_ = Bar::barcount_;
        Bar::barcount_++;
    }

    graphic_data_t Bar::Init() {
        memset(name_, ' ', 15);
        snprintf(name_, 15, "b%d", barID_);
        UserInterface::LineDraw(
            &bar_, name_, UI_Graph_Add, 0, color_, barHeight_ - 10, barStartX_ + barWidth_ / 2,
            barStartY_ + 5, barStartX_ + barWidth_ / 2, barStartY_ + barHeight_ - 5);
        return bar_;
    }

    graphic_data_t Bar::Delete() {
        UserInterface::LineDraw(
            &bar_, name_, UI_Graph_Del, 0, color_, 2, barStartX_ + 5, barStartY_ + 5,
            barStartX_ + barWidth_ - 5, barStartY_ + barHeight_ - 5);
        return bar_;
    }

    graphic_data_t Bar::InitFrame() {
        memset(name_frame_, ' ', 15);
        snprintf(name_frame_, 15, "f%d", barID_);
        UserInterface::RectangleDraw(
            &barFrame_, name_frame_, UI_Graph_Add, 0, frame_color_, 2, barStartX_, barStartY_,
            barStartX_ + barWidth_, barStartY_ + barHeight_);
        return barFrame_;
    }

    graphic_data_t Bar::DeleteFrame() {
        UserInterface::RectangleDraw(
            &barFrame_, name_frame_, UI_Graph_Del, 0, frame_color_, 2, barStartX_, barStartY_,
            barStartX_ + barWidth_, barStartY_ + barHeight_);
        return barFrame_;
    }

    graphic_data_t Bar::Update(float percent, int8_t color) {
        percent_ = percent;
        if (color != -1)
            color_ = color;
        if (isVertical_) {
            UserInterface::LineDraw(&bar_, name_, UI_Graph_Change, 0, color_,
                                                   barWidth_ - 10, barStartX_ + barWidth_ / 2,
                                                   barStartY_ + 5, barStartX_ + barWidth_ / 2,
                                                   barStartY_ + 5 + (barHeight_ - 10) * percent_);
        } else {
            UserInterface::LineDraw(
                &bar_, name_, UI_Graph_Change, 0, color_, barHeight_ - 10, barStartX_ + 5,
                barStartY_ + barHeight_ / 2, barStartX_ + 5 + (barWidth_ - 10) * percent_,
                barStartY_ + barHeight_ / 2);
        }
        return bar_;
    }
}  // namespace RMFC
