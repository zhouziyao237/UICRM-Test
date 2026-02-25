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

#include "axis.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_thread.h"

/**
 * @brief 陀螺仪数据结构体
 * @note 存储从陀螺仪传感器读取的原始数据和处理后的数据
 *
 * @param gyroADCRaw 原始ADC采样值，XYZ三轴数据
 * @param gyroADC 处理后的角速度数据，单位为度/秒
 * @param gyroZero 零点偏移校准值，用于消除静态偏差
 * @param temperature 温度传感器读数
 * @param dataReady 数据就绪标志，1表示新数据可用
 * @param Align 对齐标志，用于坐标系转换
 * @param OffsetAngle 角度偏移量，用于姿态校准
 */
typedef struct gyroData_s {
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];
    float gyroADC[XYZ_AXIS_COUNT];
    float gyroZero[XYZ_AXIS_COUNT];
    int16_t temperature;
    uint8_t dataReady;
    uint8_t Align;
    int16_t OffsetAngle[XYZ_AXIS_COUNT];
} gyroData_t;

/**
 * @brief 陀螺仪设备配置结构体
 * @note 用于存储陀螺仪和加速度计的硬件接口配置信息
 *
 * @param hspi SPI总线句柄，用于与传感器通信
 * @param CS_ACCEL 加速度计片选引脚的GPIO端口
 * @param CS_GYRO 陀螺仪片选引脚的GPIO端口
 * @param INT_ACCEL 加速度计中断引脚的GPIO端口
 * @param INT_GYRO 陀螺仪中断引脚的GPIO端口
 * @param is_DMA 是否使用DMA传输，1表示使用，0表示不使用
 */
typedef struct gyroDeviceConfig_s {
    bsp::SPIMaster* spi_master;
    bsp::GPIO* CS_ACCEL;
    bsp::GPIO* CS_GYRO;
    bsp::GPIT* INT_ACCEL = nullptr;
    bsp::GPIT* INT_GYRO = nullptr;
    bool is_DMA = true;
} gyroDeviceConfig_t;

extern gyroData_t gyroData;
extern gyroDeviceConfig_t gyroDeviceConfig;
typedef void (*gyro_callback_t)();

// todo: 后期会专门做一个硬件初始的io文件夹存放现在只是临时
extern bsp::SPI* spi1;
extern bsp::SPIMaster* spi1_master;

// ---------------------------------------- Model Main Code ----------------------------------------

extern osThreadId_t GyroTaskHandle;
const osThreadAttr_t gyroTaskAttribute = {.name = "GyroTask",
                                           .attr_bits = osThreadDetached,
                                           .cb_mem = nullptr,
                                           .cb_size = 0,
                                           .stack_mem = nullptr,
                                           .stack_size = 128 * 4,
                                           .priority = (osPriority_t)osPriorityHigh,
                                           .tz_module = 0,
                                           .reserved = 0};

void InitGyro();
void GyroCallback();
void GyroMainTask(void *args);