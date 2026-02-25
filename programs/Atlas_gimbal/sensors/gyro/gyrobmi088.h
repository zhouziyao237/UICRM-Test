// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
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
// Created by Administrator on 2026/2/24.
//

#pragma once

#include "config.h"
#include "gyro_task.h"
#include "BMI088.h"

#ifdef USE_BMI088

extern imu::BMI088* bmi088;

void InitBMI088(gyroDeviceConfig_t* config, gyro_callback_t gyro_callback);

void BMI088update(gyroData_t* gyro);

#endif
