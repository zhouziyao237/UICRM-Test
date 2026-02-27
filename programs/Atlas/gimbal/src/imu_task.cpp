/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "imu_task.h"
#include "sys.h"

// ----------------------------- IMU Data code -----------------------------

#define RX_SIGNAL (1 << 0)

control::AHRS* ahrs = nullptr;
driver::Heater* heater = nullptr;
bsp::SPIMaster* spi1_master = nullptr;
imu::BMI088* bmi088 = nullptr;
imu::IST8310* ist8310 = nullptr;
INS_Theta_t INSTheta;

// ----------------------------- IMU Task code -----------------------------

void BMI088ReceiveDone() {
    ahrs->Update(bmi088->gyro_[0], bmi088->gyro_[1], bmi088->gyro_[2], bmi088->accel_[0],
         bmi088->accel_[1], bmi088->accel_[2]);
    heater->Update(bmi088->temperature_);
    INSTheta.pitch = -ahrs->INS_angle[1];
    INSTheta.yaw = ahrs->INS_angle[0];
}

// ----------------------------- IMU Main code -----------------------------

void InitIMU() {
    HAL_Delay(500);
    bsp::spi_master_init_t spiMasterInit = {
        .spi = SPIS[HSPI1_Prot],
    };
    spi1_master = new bsp::SPIMaster(spiMasterInit);
    imu::BMI088_init_t bmi088Init = {
        .spi_master = spi1_master,
        .CS_ACCEL = GPIOS[BMI088_ACCEL_CS_Pin],
        .CS_GYRO = GPIOS[BMI_GRYO_CS_Pin],
        .INT_ACCEL = INTGPIOS[BMI088_ACCEL_INT_Pin],
        .INT_GYRO = INTGPIOS[BMI088_GRYO_INT_Pin],
        .is_DMA = true,
    };
    bmi088 = new imu::BMI088(bmi088Init);
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

    imu::IST8310_init_t ist8310Init = {.i2c = I2CS[I2C3_Prot], .drdy = INTGPIOS[IST8310_INT_Pin], .rst = GPIOS[IST8310_RST_Pin]};
    ist8310 = new imu::IST8310(ist8310Init);

    ahrs = new control::AHRS(false);
    driver::heater_init_t heater_init = {
        .pwm = PWMS[heater_pwm],
        .target_temp = 45.0f,
    };
    heater = new driver::Heater(heater_init);

    bmi088->RegisterCallback(BMI088ReceiveDone);
    osDelay(500);
}


