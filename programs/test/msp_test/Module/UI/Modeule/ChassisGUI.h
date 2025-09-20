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

#ifndef UICRM_CHASSISGUI_H
#define UICRM_CHASSISGUI_H

#include "OSD.h"

namespace RMFC {

    class ChassisGUI {
      public:
        ChassisGUI(UserInterface* UI, int16_t chassis_X = 1300, int16_t chassis_Y = 120,
                   int16_t speed_center_X = 200, int16_t speed_center_Y = 780);
        void Init();
        void Delete();
        void Init2();
        void Delete2();
        void Update(float speed_x, float speed_y, float relative);

      private:
        UserInterface* UI_;
        int16_t chassis_X_;
        int16_t chassis_Y_;
        int16_t gimbalLen_;
        int16_t chassisLen_;
        int16_t speed_center_X_;
        int16_t speed_center_Y_;
        int16_t speed_circle_R_;
        graphic_data_t chassis_;
        graphic_data_t arrow_;
        graphic_data_t gimbal_;
        graphic_data_t speed_circle_;
        graphic_data_t speed_center_;
        graphic_data_t speed_x_axis_;
        graphic_data_t speed_y_axis_;
        graphic_data_t speed_x_val_;
        graphic_data_t speed_y_val_;
    };

    class CrossairGUI {
      public:
        CrossairGUI(UserInterface* UI, int16_t centerX = 960, int16_t centerY = 540,
                    int16_t x1 = -90, int16_t x2 = -100, int16_t x3 = -110, int16_t x4 = -120,
                    int16_t x5 = -130, int16_t x6 = -140);
        void Init(int16_t x1 = -90, int16_t x2 = -100, int16_t x3 = -110, int16_t x4 = -120,
                  int16_t x5 = -130, int16_t x6 = -140);
        void Delete();

      private:
        UserInterface* UI_;
        graphic_data_t crosshair1_;
        graphic_data_t crosshair2_;
        graphic_data_t crosshair3_;
        graphic_data_t crosshair4_;
        graphic_data_t crosshair5_;
        graphic_data_t crosshair6_;
        graphic_data_t crosshair7_;
        int16_t centerX_;
        int16_t centerY_;
    };
}  // namespace RMFC

#endif  // UICRM_CHASSISGUI_H
