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

#include "gyro_task.h"
#include "gyrobmi088.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"

gyroData_t gyroData = {};
gyroDeviceConfig_t gyroDeviceConfig = {};
bsp::EventThread* gyro_callback_thread_ = nullptr;

osThreadId_t GyroTaskHandle;

// todo: 后期会专门做一个硬件初始的io文件夹存放现在只是临时
bsp::SPI* spi1 = nullptr;
bsp::SPIMaster* spi1_master = nullptr;

// ---------------------------------------- Model Main Code ----------------------------------------

void InitGyro() {

#ifdef USE_BMI088
    bsp::spi_init_t spiInit = {
        .hspi = &USE_BMI088_SPI,
        .mode = bsp::SPI_MODE_DMA,
    };
    spi1 = new bsp::SPI(spiInit);
    bsp::spi_master_init_t spiMasterInit = {
        .spi = spi1,
    };
    gyroDeviceConfig.spi_master = new bsp::SPIMaster(spiMasterInit);
    gyroDeviceConfig.CS_ACCEL = new bsp::GPIO(USE_BMI088_CS_ACCEL_PROT, USE_BMI088_CS_ACCEL_PIN);
    gyroDeviceConfig.CS_GYRO = new bsp::GPIO(USE_BMI088_CS_GYRO_PROT, USE_BMI088_CS_GYRO_PIN);
    InitBMI088(&gyroDeviceConfig, nullptr);
#endif

}


void GyroMainTask(void *args) {
    UNUSED(args);

    while (true) {

#ifdef USE_BMI088
        BMI088update(&gyroData);
#endif

    osDelay(1);
    }
}