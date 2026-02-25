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

#define USE_VTM
#define USE_VTM_UART huart6
#define USE_VTM_BANDRATE 912600

#define USE_BMI088
#define USE_BMI088_SPI hspi1
#define USE_BMI088_CS_ACCEL_PROT CS1_ACCEL_GPIO_Port
#define USE_BMI088_CS_ACCEL_PIN CS1_ACCEL_Pin
#define USE_BMI088_CS_GYRO_PROT CS1_GYRO_GPIO_Port
#define USE_BMI088_CS_GYRO_PIN CS1_GYRO_Pin
#define USE_BMI088_CS_ACCEL_INIT INT1_ACCEL_Pin
#define USE_BMI088_CS_GYRO_INIT INT1_GYRO_Pin
#define USE_BMI088_DMA false

#define MAX_SUPPORTED_RC_CHANNEL_COUNT 26
