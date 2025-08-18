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
// Created by Administrator on 2025/8/1.
//

#include "main.h"

#include "Common/config.h"
#include "Device/bsp_SPI/bsp_SPIBus.h"
#include "Device/bsp_serial/DeviceSerial.h"
#include "Device/bsp_serial/bsp_Serial.h"
#include "MPU6500.h"
#include "Msp/MspSerial.h"
#include "MspProcessor.h"
#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_pg/bsp_pg.h"
#include "bsp_print.h"
#include "bsp_serial.h"
#include "bsp_spi.h"
#include "cmsis_os2.h"

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6

static imu::MPU6500* mpu6500 = nullptr;
static bsp::GPIO* imu_cs = nullptr;
static bsp::GPIT* mpu6500_it = nullptr;
static bsp::SPI* spi5 = nullptr;
static bsp::SPIMaster* spi5_master = nullptr;

serialPort_t* serialmsp;
RMFC::MSP::MspProcessor _msp;
RMFC::MSP::MspMessage mspRequest;
RMFC::MSP::MspResponse mspResponse;

void InitDebugMPU() {
    imu_cs = new bsp::GPIO(ONBOARD_IMU_CS_GROUP, ONBOARD_IMU_CS_PIN);
    mpu6500_it = new bsp::GPIT(MPU6500_IT_Pin);

    bsp::spi_init_t spi5_init{.hspi = &ONBOARD_IMU_SPI, .mode = bsp::SPI_MODE_DMA};
    spi5 = new bsp::SPI(spi5_init);

    bsp::spi_master_init_t spi5_master_init = {
        .spi = spi5,
    };
    spi5_master = new bsp::SPIMaster(spi5_master_init);

    imu::mpu6500_init_t mpu6500_init;
    mpu6500_init.spi = spi5_master;
    mpu6500_init.cs = imu_cs;
    mpu6500_init.int_pin = mpu6500_it;
    mpu6500_init.use_mag = true;
    mpu6500_init.dma = true;

    mpu6500 = new imu::MPU6500(mpu6500_init);
    bsp::SetHighresClockTimer(&htim7);
}

void MPU_Updata() {
    set_cursor(0, 0);
    clear_screen();
    print("Temp: %10.4f\r\n", mpu6500->temperature_);
    print("ACC_X: %9.4f ACC_Y: %9.4f ACC_Z: %9.4f\r\n", mpu6500->accel_[0], mpu6500->accel_[1],
          mpu6500->accel_[2]);
    print("GYRO_X: %8.4f GYRO_Y: %8.4f GYRO_Z: %8.4f\r\n", mpu6500->gyro_[0], mpu6500->gyro_[1],
          mpu6500->gyro_[2]);
    print("MAG_X: %9.0f MAG_Y: %9.0f MAG_Z: %9.0f\r\n", mpu6500->mag_[0], mpu6500->mag_[1],
          mpu6500->mag_[2]);
    print("\r\nTime Stamp: %.2f us\r\n", mpu6500->time_);
    osDelay(100);
}

void RM_RTOS_Init() {
    pgResetAll();

#ifdef USE_DEBUGUART
    print_use_uart(USE_DEBUGUART, false, 912600);
#endif

    print("EEPROM start: 0x%08lX\r\n", (uint32_t)__config_start);
    print("EEPROM end: 0x%08lX\r\n", (uint32_t)__config_end);
    print("EEPROM size: %lu bytes\r\n", (uint32_t)(__config_end - __config_start));

    initEEPROM();
    ensureEEPROMStructureIsValid();
    readEEPROM();
    InitDebugMPU();

    RMFC::serialInit(false, RMFC::SERIAL_PORT_NONE);
    RMFC::MSP::mspSerialInit();
    osDelay(100);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    while (true) {
        RMFC::MSP::mspSerialProcess(RMFC::MSP::MSP_EVALUATE_NON_MSP_DATA);
        MPU_Updata();
    }
}
