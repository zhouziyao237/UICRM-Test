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

#ifndef UICRM_GIMBALGUI_H
#define UICRM_GIMBALGUI_H

#include "OSD.h"
#include "Bar.h"

namespace RMFC {
    class GimbalGUI {
      public:
        GimbalGUI(UserInterface* UI, int16_t gimbal_speed_center_X = 1620,
                  int16_t gimbal_speed_center_Y = 780, int16_t gimbal_speed_center_R = 100,
                  int16_t pitch_bar_X = 1800, int16_t pitch_bar_Y = 440,
                  int16_t pitch_bar_height = 400, int16_t pitch_bar_weight = 30,
                  float pitch_max = 0.4253f);
        ~GimbalGUI();
        void Init();
        void Init2();
        void Delete();
        void Delete2();
        void Update(float vpitch, float vyaw, float pitch, float yaw, bool flags);

      private:
        UserInterface* UI_;
        int16_t gimbal_speed_center_X_;
        int16_t gimbal_speed_center_Y_;
        int16_t gimbal_speed_circle_R_;
        int16_t pitch_bar_X_;
        int16_t pitch_bar_Y_;
        int16_t pitch_bar_height_;
        int16_t pitch_bar_weight_;
        float pitch_max_;
        graphic_data_t speed_circle_;
        graphic_data_t speed_center_;
        graphic_data_t speed_x_axis_;
        graphic_data_t speed_y_axis_;
        graphic_data_t speed_x_val_;
        graphic_data_t speed_y_val_;
        graphic_data_t calibration_flag_;
        Bar* pitch_bar_;
        graphic_data_t pitch_bar_frame_;
        graphic_data_t pitch_bar_val_;
    };
}  // namespace RMFC

#endif  // UICRM_GIMBALGUI_H
