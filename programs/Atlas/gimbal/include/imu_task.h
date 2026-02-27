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
// Created by Administrator on 2025/3/15.
//

#pragma once

#include "AHRS.h"
#include "BMI088.h"
#include "IST8310.h"
#include "heater.h"
#include "cmsis_os.h"
#include "main.h"

// ----------------------------- IMU Data code -----------------------------
#define RX_SIGNAL (1 << 0)

extern control::AHRS* ahrs;
extern driver::Heater* heater;
extern bsp::SPIMaster* spi1_master;
extern imu::BMI088* bmi088;
extern imu::IST8310* ist8310;
typedef struct {
    float pitch;
    float roll;
    float yaw;
} INS_Theta_t;
extern INS_Theta_t INSTheta;

// ----------------------------- IMU Main code -----------------------------
void InitIMU();