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

#include "GimbalGUI.h"

#include "Bar.h"
#include "OSD.h"

namespace RMFC {
    GimbalGUI::GimbalGUI(UserInterface* UI, int16_t gimbal_speed_center_X,
                         int16_t gimbal_speed_center_Y, int16_t gimbal_speed_circle_R,
                         int16_t pitch_bar_X, int16_t pitch_bar_Y, int16_t pitch_bar_height,
                         int16_t pitch_bar_weight, float pitch_max)
        : UI_(UI) {
        gimbal_speed_center_X_ = gimbal_speed_center_X;
        gimbal_speed_center_Y_ = gimbal_speed_center_Y;
        gimbal_speed_circle_R_ = gimbal_speed_circle_R;
        pitch_bar_X_ = pitch_bar_X;
        pitch_bar_Y_ = pitch_bar_Y;
        pitch_bar_height_ = pitch_bar_height;
        pitch_bar_weight_ = pitch_bar_weight;
        pitch_max_ = pitch_max;
        pitch_bar_ = new Bar(pitch_bar_X_, pitch_bar_Y_, pitch_bar_weight_, pitch_bar_height_,
                             UI_Color_Green, UI_Color_Pink, true);
        Init();
    }

    GimbalGUI::~GimbalGUI() {
        delete pitch_bar_;
    }
    void GimbalGUI::Init() {
        UI_->CircleDraw(&speed_circle_, "gc", UI_Graph_Add, 1, UI_Color_Yellow, 2,
                        gimbal_speed_center_X_, gimbal_speed_center_Y_, gimbal_speed_circle_R_);
        UI_->CircleDraw(&speed_center_, "gd", UI_Graph_Add, 2, UI_Color_Green, 10,
                        gimbal_speed_center_X_, gimbal_speed_center_Y_, 5);
        UI_->LineDraw(&speed_x_axis_, "gx", UI_Graph_Add, 3, UI_Color_Pink, 2,
                      gimbal_speed_center_X_ - gimbal_speed_circle_R_, gimbal_speed_center_Y_,
                      gimbal_speed_center_X_ + gimbal_speed_circle_R_, gimbal_speed_center_Y_);
        UI_->LineDraw(&speed_y_axis_, "gy", UI_Graph_Add, 3, UI_Color_Pink, 2,
                      gimbal_speed_center_X_, gimbal_speed_center_Y_ - gimbal_speed_circle_R_,
                      gimbal_speed_center_X_, gimbal_speed_center_Y_ + gimbal_speed_circle_R_);
        UI_->CircleDraw(&calibration_flag_, "cal", UI_Graph_Add, 0, UI_Color_Pink, 14, 960 + 925,
                        540 + 330, 7);
        UI_->IntDraw(&speed_x_val_, "gvx", UI_Graph_Add, 2, UI_Color_Green, 10, 2,
                     gimbal_speed_center_X_ + gimbal_speed_circle_R_ + 8,
                     gimbal_speed_center_Y_ - 2, 0);
        UI_->IntDraw(&speed_y_val_, "gvy", UI_Graph_Add, 2, UI_Color_Green, 10, 2,
                     gimbal_speed_center_X_ - 5,
                     gimbal_speed_center_Y_ - gimbal_speed_circle_R_ - 10, 0);
        UI_->GraphRefresh(7, speed_circle_, speed_center_, speed_x_axis_, speed_y_axis_,
                          speed_x_val_, speed_y_val_, calibration_flag_);
    }

    void GimbalGUI::Delete() {
        UI_->CircleDraw(&speed_circle_, "gc", UI_Graph_Del, 1, UI_Color_Yellow, 2,
                        gimbal_speed_center_X_, gimbal_speed_center_Y_, gimbal_speed_circle_R_);
        UI_->CircleDraw(&speed_center_, "gd", UI_Graph_Del, 2, UI_Color_Green, 10,
                        gimbal_speed_center_X_, gimbal_speed_center_Y_, 5);
        UI_->LineDraw(&speed_x_axis_, "gx", UI_Graph_Del, 3, UI_Color_Pink, 2,
                      gimbal_speed_center_X_ - gimbal_speed_circle_R_, gimbal_speed_center_Y_,
                      gimbal_speed_center_X_ + gimbal_speed_circle_R_, gimbal_speed_center_Y_);
        UI_->LineDraw(&speed_y_axis_, "gy", UI_Graph_Del, 3, UI_Color_Pink, 2,
                      gimbal_speed_center_X_, gimbal_speed_center_Y_ - gimbal_speed_circle_R_,
                      gimbal_speed_center_X_, gimbal_speed_center_Y_ + gimbal_speed_circle_R_);
        UI_->CircleDraw(&calibration_flag_, "cal", UI_Graph_Del, 0, UI_Color_Pink, 14, 960 + 925,
                        540 + 330, 7);
        UI_->IntDraw(&speed_x_val_, "gvx", UI_Graph_Del, 2, UI_Color_Green, 10, 2,
                     gimbal_speed_center_X_ + gimbal_speed_circle_R_ + 8,
                     gimbal_speed_center_Y_ - 2, 0);
        UI_->IntDraw(&speed_y_val_, "gvy", UI_Graph_Del, 2, UI_Color_Green, 10, 2,
                     gimbal_speed_center_X_ - 5,
                     gimbal_speed_center_Y_ - gimbal_speed_circle_R_ - 10, 0);
        UI_->GraphRefresh(7, speed_circle_, speed_center_, speed_x_axis_, speed_y_axis_,
                          speed_x_val_, speed_y_val_, calibration_flag_);
    }

    void GimbalGUI::Init2() {
        pitch_bar_val_ = pitch_bar_->Init();
        pitch_bar_frame_ = pitch_bar_->InitFrame();
        UI_->GraphRefresh(2, pitch_bar_frame_, pitch_bar_val_);
    }
    void GimbalGUI::Delete2() {
        pitch_bar_val_ = pitch_bar_->Delete();
        pitch_bar_frame_ = pitch_bar_->DeleteFrame();
        UI_->GraphRefresh(2, pitch_bar_frame_, pitch_bar_val_);
    }

    void GimbalGUI::Update(float vpitch, float vyaw, float pitch, float yaw, bool flags) {
        UNUSED(yaw);
        vpitch = clip<float>(vpitch, -1, 1);
        vyaw = clip<float>(vyaw, -1, 1);
        if (flags) {
            UI_->CircleDraw(&calibration_flag_, "cal", UI_Graph_Change, 0, UI_Color_Green, 14,
                            960 + 925, 540 + 330, 7);
        } else {
            UI_->CircleDraw(&calibration_flag_, "cal", UI_Graph_Change, 0, UI_Color_Pink, 14,
                            960 + 925, 540 + 330, 7);
        }
        UI_->CircleDraw(&speed_center_, "gd", UI_Graph_Change, 2, UI_Color_Green, 10,
                        gimbal_speed_center_X_ + (int16_t)(vyaw * gimbal_speed_circle_R_),
                        gimbal_speed_center_Y_ + (int16_t)(vpitch * gimbal_speed_circle_R_), 5);
        UI_->IntDraw(&speed_x_val_, "gvx", UI_Graph_Change, 2, UI_Color_Green, 10, 2,
                     gimbal_speed_center_X_ + gimbal_speed_circle_R_ + 8,
                     gimbal_speed_center_Y_ - 2, (int32_t)(vyaw * 100));
        UI_->IntDraw(&speed_y_val_, "gvy", UI_Graph_Change, 2, UI_Color_Green, 10, 2,
                     gimbal_speed_center_X_ - 5,
                     gimbal_speed_center_Y_ - gimbal_speed_circle_R_ - 10, (int32_t)(vpitch * 100));
        float pitch_percent = 1 - (pitch + pitch_max_) / (2 * pitch_max_);
        pitch_percent = clip<float>(pitch_percent, 0, 1);
        pitch_bar_val_ = pitch_bar_->Update(pitch_percent);
        UI_->GraphRefresh(5, speed_center_, speed_x_val_, speed_y_val_, calibration_flag_,
                          pitch_bar_val_);
    }
}  // namespace RMFC