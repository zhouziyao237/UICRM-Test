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

#include "gyrobmi088.h"
#include "gyro_task.h"
#include <cstring>

#ifdef USE_BMI088

imu::BMI088* bmi088 = nullptr;

void InitBMI088(gyroDeviceConfig_t* config, gyro_callback_t gyro_callback) {
    if (!config)
        RM_ASSERT_TRUE(false, "gyroDeviceConfig is null");

    config->INT_ACCEL = new bsp::GPIT(USE_BMI088_CS_ACCEL_INIT);
    config->INT_GYRO = new bsp::GPIT(USE_BMI088_CS_GYRO_INIT);

    imu::BMI088_init_t bmi088Init = {
        .spi_master = config->spi_master,
        .CS_ACCEL = config->CS_ACCEL,
        .CS_GYRO = config->CS_GYRO,
        .INT_ACCEL = config->INT_ACCEL,
        .INT_GYRO = config->INT_GYRO,
        .is_DMA = USE_BMI088_DMA,
        .use_thread_ = false,
    };

    bmi088 = new imu::BMI088(bmi088Init);
    if (gyro_callback != nullptr) {
        bmi088->RegisterCallback(reinterpret_cast<imu::BMI088_callback_t>(gyro_callback));
    }
}

void BMI088update(gyroData_t* gyro) {
    if (!gyro || !bmi088) {
        return;
    }

    if (bmi088->IsReady()) {
        // 更新陀螺仪数据
        gyro->gyroADCRaw[0] = static_cast<int16_t>(bmi088->gyro_[0]);
        gyro->gyroADCRaw[1] = static_cast<int16_t>(bmi088->gyro_[1]);
        gyro->gyroADCRaw[2] = static_cast<int16_t>(bmi088->gyro_[2]);

        // 更新加速度计数据
        gyro->gyroADC[0] = bmi088->accel_[0];
        gyro->gyroADC[1] = bmi088->accel_[1];
        gyro->gyroADC[2] = bmi088->accel_[2];

        // 更新温度
        gyro->temperature = static_cast<int16_t>(bmi088->temperature_);
        gyro->dataReady = 1;
    }
}

#endif

