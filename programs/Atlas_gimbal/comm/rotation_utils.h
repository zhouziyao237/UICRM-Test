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
#include "bsp_math.h"

#define GET_AXIS_ROT(bits, axis) ((bits & ROT_AXIS_MASK(axis)) >> ((FD_YAW - axis) * ROT_WIDTH))

#define ALIGN_TO_BITS(align) ((((align) - CW0_DEG) & 0x3) | ((((align) - CW0_DEG) & 0x4) << 1))

/**
 * @brief 传感器安装方向枚举
 */
typedef enum {
    ALIGN_DEF = 0,
    CW0_DEG = 1,
    CW90_DEG = 2,
    CW180_DEG = 3,
    CW270_DEG = 4,
    CW0_FLIP = 5,
    CW90_FLIP = 6,
    CW180_FLIP = 7,
    CW270_FLIP = 8,
    ALIGN_CUSTOM = 9,
} sensor_align_e;

/**
 * @brief 传感器对齐结构体
 */
typedef union sensor_align_u {
    float raw[XYZ_AXIS_COUNT];
    struct {
        float roll;
        float pitch;
        float yaw;
    };
} sensor_align_t;

// ==================== 宏定义简化接口 ====================

/**
 * @brief 度转弧度宏
 */
#define DEG_TO_RAD(deg) ((deg) * MATH_PI / 180.0f)

/**
 * @brief 弧度转度宏
 */
#define RAD_TO_DEG(rad) ((rad) * 180.0f / MATH_PI)

/**
 * @brief 快速创建欧拉角（度单位）
 */
#define EULER_DEG(roll, pitch, yaw)                          \
    (euler_t) {                                              \
        DEG_TO_RAD(roll), DEG_TO_RAD(pitch), DEG_TO_RAD(yaw) \
    }

/**
 * @brief 快速创建欧拉角（弧度单位）
 */
#define EULER_RAD(roll, pitch, yaw) \
    (euler_t) {                     \
        roll, pitch, yaw            \
    }

/**
 * @brief 快速创建传感器对齐（度单位）
 */
#define SENSOR_ALIGN_DEG(roll, pitch, yaw) \
    (sensor_align_t) {                     \
        roll, pitch, yaw                   \
    }

/**
 * @brief 快速创建传感器对齐（弧度单位）
 */
#define SENSOR_ALIGN_RAD(roll, pitch, yaw)                   \
    (sensor_align_t) {                                       \
        RAD_TO_DEG(roll), RAD_TO_DEG(pitch), RAD_TO_DEG(yaw) \
    }

/**
 * @brief 度单位欧拉角直接转矩阵
 */
#define EULER_DEG_TO_MAT(roll, pitch, yaw) \
    euler_to_mat(&(euler_t){DEG_TO_RAD(roll), DEG_TO_RAD(pitch), DEG_TO_RAD(yaw)})

/**
 * @brief 弧度单位欧拉角直接转矩阵
 */
#define EULER_RAD_TO_MAT(roll, pitch, yaw) euler_to_mat(&(euler_t){roll, pitch, yaw})

#define SENSOR_ALIGN(ROLL, PITCH, YAW) \
    ((const sensor_align_t){           \
        .roll = (ROLL),                \
        .pitch = (PITCH),              \
        .yaw = (YAW),                  \
    })

#define CUSTOM_CW(deg) SENSOR_ALIGN(0, 0, (deg))
#define CUSTOM_CW_FLIP(deg) SENSOR_ALIGN(0, 180, (deg))

/**
 * @brief 欧拉角结构体
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} euler_t;

/**
 * @brief 3x3旋转矩阵结构体
 */
typedef struct {
    float m[3][3];
} rot_mat_t;

// ==================== 核心变换函数 ====================

/**
 * @brief 欧拉角转旋转矩阵
 */
rot_mat_t euler_to_mat(const euler_t* euler);

/**
 * @brief 旋转矩阵转欧拉角
 */
euler_t mat_to_euler(const rot_mat_t* matrix);

/**
 * @brief 点正向变换
 */
Vec3f point_forward(const rot_mat_t* matrix, const Vec3f* point);

/**
 * @brief 点反向变换
 */
Vec3f point_backward(const rot_mat_t* matrix, const Vec3f* point);

/**
 * @brief 批量点变换
 */
void points_batch(const rot_mat_t* matrix, const Vec3f* in, Vec3f* out, uint32_t count);

// ==================== 传感器对齐函数 ====================

/**
 * @brief 对齐枚举转欧拉角
 */
euler_t align_to_euler(sensor_align_e align);

/**
 * @brief 设置传感器对齐（直接输入值）
 */
void set_sensor_align(sensor_align_t* align, float roll, float pitch, float yaw);

/**
 * @brief 对齐转旋转矩阵
 */
rot_mat_t align_to_mat(const sensor_align_t* align);

/**
 * @brief 组合旋转矩阵
 */
rot_mat_t combine_mats(const rot_mat_t* r1, const rot_mat_t* r2);

/**
 * @brief 验证旋转矩阵
 */
bool is_valid_mat(const rot_mat_t* matrix);
