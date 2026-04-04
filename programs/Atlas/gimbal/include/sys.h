#pragma once

#include "atlas_config.h"
#include "robot_state.h"

#include "bsp_can.h"
#include "bsp_gpio.h"
#include "bsp_i2c.h"
#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "bsp_thread.h"
#include "bsp_uart.h"
#include "bsp_usb.h"

/** Named hardware handles. No numeric application-level device indexes. */
extern bsp::CAN* atlas_can1;
extern bsp::CAN* atlas_can2;
extern bsp::UART* atlas_referee_uart;
extern bsp::UART* atlas_log_uart;
extern bsp::VirtualUSB* atlas_minipc_usb;

extern bsp::SPI* atlas_imu_spi;
extern bsp::I2C* atlas_magnetometer_i2c;
extern bsp::GPIO* atlas_imu_accel_cs;
extern bsp::GPIO* atlas_imu_gyro_cs;
extern bsp::GPIO* atlas_magnetometer_reset;
extern bsp::GPIT* atlas_imu_accel_interrupt;
extern bsp::GPIT* atlas_imu_gyro_interrupt;
extern bsp::GPIT* atlas_magnetometer_interrupt;
extern bsp::PWM* atlas_imu_heater_pwm;

void InitAtlasHardware();
void SysTask(void* arg);

extern osThreadId_t SysTaskHandle;
inline const osThreadAttr_t SysTaskAttribute = {
    .name = "SysTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 512 * 2,
    .priority = (osPriority_t)osPriorityHigh5,
    .tz_module = 0,
    .reserved = 0,
};
