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

#include "ChassisGUI.h"

#include "OSD.h"

namespace RMFC {
    ChassisGUI::ChassisGUI(UserInterface* UI, int16_t chassis_X, int16_t chassis_Y,
                           int16_t speed_center_X, int16_t speed_center_Y)
        : UI_(UI) {
        chassis_X_ = chassis_X;
        chassis_Y_ = chassis_Y;
        speed_center_X_ = speed_center_X;
        speed_center_Y_ = speed_center_Y;
        gimbalLen_ = 90;
        chassisLen_ = 90;
        speed_circle_R_ = 100;
        Init();
    }
    void ChassisGUI::Init() {
        UI_->LineDraw(&chassis_, "c", UI_Graph_Add, 1, UI_Color_Yellow, 60, chassis_X_,
                      chassis_Y_ - chassisLen_ / 2, chassis_X_, chassis_Y_ + chassisLen_ / 2);
        UI_->LineDraw(&arrow_, "a", UI_Graph_Add, 1, UI_Color_Yellow, 20, chassis_X_ - 7,
                      chassis_Y_ + chassisLen_ / 2 - 7, chassis_X_ + 7,
                      chassis_Y_ + chassisLen_ / 2 + 7);
        UI_->LineDraw(&gimbal_, "g", UI_Graph_Add, 2, UI_Color_White, 7, chassis_X_, chassis_Y_,
                      chassis_X_, chassis_Y_ + gimbalLen_);
        UI_->CircleDraw(&speed_circle_, "sc", UI_Graph_Add, 1, UI_Color_Yellow, 2, speed_center_X_,
                        speed_center_Y_, speed_circle_R_);
        UI_->CircleDraw(&speed_center_, "sd", UI_Graph_Add, 2, UI_Color_Green, 10, speed_center_X_,
                        speed_center_Y_, 5);
        UI_->LineDraw(&speed_x_axis_, "sx", UI_Graph_Add, 3, UI_Color_Pink, 2,
                      speed_center_X_ - speed_circle_R_, speed_center_Y_,
                      speed_center_X_ + speed_circle_R_, speed_center_Y_);
        UI_->LineDraw(&speed_y_axis_, "sy", UI_Graph_Add, 3, UI_Color_Pink, 2, speed_center_X_,
                      speed_center_Y_ - speed_circle_R_, speed_center_X_,
                      speed_center_Y_ + speed_circle_R_);
        UI_->GraphRefresh(7, chassis_, arrow_, gimbal_, speed_circle_, speed_center_, speed_x_axis_,
                          speed_y_axis_);
    }

    void ChassisGUI::Delete() {
        UI_->LineDraw(&chassis_, "c", UI_Graph_Del, 1, UI_Color_Yellow, 60, chassis_X_,
                      chassis_Y_ - chassisLen_ / 2, chassis_X_, chassis_Y_ + chassisLen_ / 2);
        UI_->LineDraw(&arrow_, "a", UI_Graph_Del, 1, UI_Color_Yellow, 20, chassis_X_ - 7,
                      chassis_Y_ + chassisLen_ / 2 - 7, chassis_X_ + 7,
                      chassis_Y_ + chassisLen_ / 2 + 7);
        UI_->LineDraw(&gimbal_, "g", UI_Graph_Del, 2, UI_Color_White, 7, chassis_X_, chassis_Y_,
                      chassis_X_, chassis_Y_ + gimbalLen_);
        UI_->CircleDraw(&speed_circle_, "sc", UI_Graph_Del, 1, UI_Color_Yellow, 2, speed_center_X_,
                        speed_center_Y_, speed_circle_R_);
        UI_->CircleDraw(&speed_center_, "sd", UI_Graph_Del, 2, UI_Color_Green, 10, speed_center_X_,
                        speed_center_Y_, 5);
        UI_->LineDraw(&speed_x_axis_, "sx", UI_Graph_Del, 3, UI_Color_Pink, 2,
                      speed_center_X_ - speed_circle_R_, speed_center_Y_,
                      speed_center_X_ + speed_circle_R_, speed_center_Y_);
        UI_->LineDraw(&speed_y_axis_, "sy", UI_Graph_Del, 3, UI_Color_Pink, 2, speed_center_X_,
                      speed_center_Y_ - speed_circle_R_, speed_center_X_,
                      speed_center_Y_ + speed_circle_R_);
        UI_->GraphRefresh(7, chassis_, arrow_, gimbal_, speed_circle_, speed_center_, speed_x_axis_,
                          speed_y_axis_);
    }

    void ChassisGUI::Init2() {
        UI_->IntDraw(&speed_x_val_, "vx", UI_Graph_Add, 2, UI_Color_Green, 10, 2,
                     speed_center_X_ + speed_circle_R_ + 8, speed_center_Y_ - 2, 0);
        UI_->IntDraw(&speed_y_val_, "vy", UI_Graph_Add, 2, UI_Color_Green, 10, 2,
                     speed_center_X_ - 5, speed_center_Y_ - speed_circle_R_ - 10, 0);
        UI_->GraphRefresh(2, speed_x_val_, speed_y_val_);
    }
    void ChassisGUI::Delete2() {
        UI_->IntDraw(&speed_x_val_, "vx", UI_Graph_Del, 2, UI_Color_Green, 10, 2,
                     speed_center_X_ + speed_circle_R_ + 8, speed_center_Y_ - 2, 0);
        UI_->IntDraw(&speed_y_val_, "vy", UI_Graph_Del, 2, UI_Color_Green, 10, 2,
                     speed_center_X_ - 5, speed_center_Y_ - speed_circle_R_ - 10, 0);
        UI_->GraphRefresh(2, speed_x_val_, speed_y_val_);
    }

    void ChassisGUI::Update(float speed_x, float speed_y, float relative) {
        float x_end = chassis_X_ - chassisLen_ / 2.0 * arm_sin_f32(relative);
        float y_end = chassis_Y_ + chassisLen_ / 2.0 * arm_cos_f32(relative);
        float x_start = chassis_X_ + chassisLen_ / 2.0 * arm_sin_f32(relative);
        float y_start = chassis_Y_ - chassisLen_ / 2.0 * arm_cos_f32(relative);
        UI_->LineDraw(&chassis_, "c", UI_Graph_Change, 1, UI_Color_Yellow, 60, (uint32_t)x_start,
                      (uint32_t)y_start, (uint32_t)x_end, (uint32_t)y_end);
        UI_->LineDraw(&arrow_, "a", UI_Graph_Change, 1, UI_Color_Yellow, 20,
                      (uint32_t)(x_end + 10 * arm_sin_f32(relative + M_PI / 4)),
                      (uint32_t)(y_end - 10 * arm_cos_f32(relative + M_PI / 4)),
                      (uint32_t)(x_end - 10 * arm_sin_f32(relative + M_PI / 4)),
                      (uint32_t)(y_end + 10 * arm_cos_f32(relative + M_PI / 4)));
        UI_->CircleDraw(&speed_center_, "sd", UI_Graph_Change, 2, UI_Color_Green, 10,
                        speed_center_X_ + (int16_t)(speed_x * speed_circle_R_),
                        speed_center_Y_ + (int16_t)(speed_y * speed_circle_R_), 5);
        UI_->IntDraw(&speed_x_val_, "vx", UI_Graph_Change, 2, UI_Color_Green, 10, 2,
                     speed_center_X_ + speed_circle_R_ + 8, speed_center_Y_ - 2,
                     (int32_t)(speed_x * 100.0f));
        UI_->IntDraw(&speed_y_val_, "vy", UI_Graph_Change, 2, UI_Color_Green, 10, 2,
                     speed_center_X_ - 5, speed_center_Y_ - speed_circle_R_ - 10,
                     (int32_t)(speed_y * 100.0f));
        UI_->LineDraw(&gimbal_, "g", UI_Graph_Change, 2, UI_Color_White, 7, chassis_X_, chassis_Y_,
                      chassis_X_, chassis_Y_ + gimbalLen_);
        // Static element that can be remove
        UI_->CircleDraw(&speed_circle_, "sc", UI_Graph_Change, 1, UI_Color_Yellow, 2,
                        speed_center_X_, speed_center_Y_, speed_circle_R_);

        UI_->GraphRefresh(7, chassis_, arrow_, gimbal_, speed_circle_, speed_center_, speed_x_val_,
                          speed_y_val_);
    }

    CrossairGUI::CrossairGUI(UserInterface* UI, int16_t centerX, int16_t centerY, int16_t x1,
                             int16_t x2, int16_t x3, int16_t x4, int16_t x5, int16_t x6)
        : UI_(UI) {
        centerX_ = centerX;
        centerY_ = centerY;
        Init(x1, x2, x3, x4, x5, x6);
    }

    void CrossairGUI::Init(int16_t x1, int16_t x2, int16_t x3, int16_t x4, int16_t x5, int16_t x6) {
        UI_->LineDraw(&crosshair1_, "ch1", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 50,
                      centerY_ + x1, centerX_ + 50, centerY_ + x1);
        UI_->LineDraw(&crosshair2_, "ch2", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 30,
                      centerY_ + x2, centerX_ + 30, centerY_ + x2);
        UI_->LineDraw(&crosshair3_, "ch3", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 30,
                      centerY_ + x3, centerX_ + 30, centerY_ + x3);
        UI_->LineDraw(&crosshair4_, "ch4", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 50,
                      centerY_ + x4, centerX_ + 50, centerY_ + x4);
        UI_->LineDraw(&crosshair5_, "ch5", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 30,
                      centerY_ + x5, centerX_ + 30, centerY_ + x5);
        UI_->LineDraw(&crosshair6_, "ch6", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_ - 30,
                      centerY_ + x6, centerX_ + 30, centerY_ + x6);
        UI_->LineDraw(&crosshair7_, "ch7", UI_Graph_Add, 0, UI_Color_Cyan, 2, centerX_,
                      centerY_ + x1 + 30, centerX_, centerY_ + x6 - 30);
        UI_->GraphRefresh(7, crosshair1_, crosshair2_, crosshair3_, crosshair4_, crosshair5_,
                          crosshair6_, crosshair7_);
    }

    void CrossairGUI::Delete() {
        UI_->LineDraw(&crosshair1_, "ch1", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_ - 50,
                      centerY_ - 40, centerX_ + 50, centerY_ - 40);
        UI_->LineDraw(&crosshair2_, "ch2", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_ - 30,
                      centerY_ - 55, centerX_ + 30, centerY_ - 55);
        UI_->LineDraw(&crosshair3_, "ch3", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_ - 30,
                      centerY_ - 60, centerX_ + 30, centerY_ - 60);
        UI_->LineDraw(&crosshair4_, "ch4", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_ - 50,
                      centerY_ - 70, centerX_ + 50, centerY_ - 70);
        UI_->LineDraw(&crosshair5_, "ch5", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_ - 30,
                      centerY_ - 80, centerX_ + 30, centerY_ - 80);
        UI_->LineDraw(&crosshair6_, "ch6", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_ - 30,
                      centerY_ - 90, centerX_ + 30, centerY_ - 90);
        UI_->LineDraw(&crosshair7_, "ch7", UI_Graph_Del, 0, UI_Color_Cyan, 2, centerX_,
                      centerY_ - 30, centerX_, centerY_ - 100);
        UI_->GraphRefresh(7, crosshair1_, crosshair2_, crosshair3_, crosshair4_, crosshair5_,
                          crosshair6_, crosshair7_);
    }
}  // namespace RMFC
