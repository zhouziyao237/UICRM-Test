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

#include "rotation_utils.h"
#include "bsp_math.h"
#include <string.h>

// ==================== 核心实现 ====================

rot_mat_t euler_to_mat(const euler_t* euler)
{
    rot_mat_t matrix = {};

    if (!euler) {
        matrix.m[0][0] = 1.0f;
        matrix.m[1][1] = 1.0f;
        matrix.m[2][2] = 1.0f;
        return matrix;
    }

    float cr = MathCos(euler->roll);
    float sr = MathSin(euler->roll);
    float cp = MathCos(euler->pitch);
    float sp = MathSin(euler->pitch);
    float cy = MathCos(euler->yaw);
    float sy = MathSin(euler->yaw);

    // 优化计算
    float t1 = cy * cp;
    float t2 = sy * cr;
    float t3 = sy * sr;

    matrix.m[0][0] = t1;
    matrix.m[0][1] = t1 * sr - t2;
    matrix.m[0][2] = t1 * cr + t3;

    float t4 = sy * cp;
    float t5 = cy * cr;
    float t6 = cy * sr;

    matrix.m[1][0] = t4;
    matrix.m[1][1] = t4 * sr + t5;
    matrix.m[1][2] = t4 * cr - t6;

    matrix.m[2][0] = -sp;
    matrix.m[2][1] = cp * sr;
    matrix.m[2][2] = cp * cr;

    return matrix;
}

euler_t mat_to_euler(const rot_mat_t* matrix)
{
    euler_t euler = {};

    if (!matrix) return euler;

    const float eps = 1e-6f;

    if (MathAbs(MathAbs(matrix->m[2][0]) - 1.0f) > eps) {
        euler.roll = MathAtan2(matrix->m[2][1], matrix->m[2][2]);
        euler.pitch = -MathAsin(matrix->m[2][0]);
        euler.yaw = MathAtan2(matrix->m[1][0], matrix->m[0][0]);
    } else {
        euler.roll = 0.0f;
        euler.pitch = MathSign(-matrix->m[2][0]) * MATH_PI_2;
        euler.yaw = MathAtan2(
            -MathSign(matrix->m[2][0]) * matrix->m[0][1],
            -MathSign(matrix->m[2][0]) * matrix->m[0][2]
        );
    }

    return euler;
}

Vec3f point_forward(const rot_mat_t* matrix, const Vec3f* point)
{
    Vec3f result = {};

    if (!matrix || !point) return result;

    result.x = Vec3fDot(VEC3F(matrix->m[0][0], matrix->m[0][1], matrix->m[0][2]), *point);
    result.y = Vec3fDot(VEC3F(matrix->m[1][0], matrix->m[1][1], matrix->m[1][2]), *point);
    result.z = Vec3fDot(VEC3F(matrix->m[2][0], matrix->m[2][1], matrix->m[2][2]), *point);

    return result;
}

Vec3f point_backward(const rot_mat_t* matrix, const Vec3f* point)
{
    Vec3f result = {};

    if (!matrix || !point) return result;

    result.x = matrix->m[0][0] * point->x + matrix->m[1][0] * point->y + matrix->m[2][0] * point->z;
    result.y = matrix->m[0][1] * point->x + matrix->m[1][1] * point->y + matrix->m[2][1] * point->z;
    result.z = matrix->m[0][2] * point->x + matrix->m[1][2] * point->y + matrix->m[2][2] * point->z;

    return result;
}

void points_batch(const rot_mat_t* matrix, const Vec3f* in, Vec3f* out, uint32_t count)
{
    if (!matrix || !in || !out || count == 0) return;

    for (uint32_t i = 0; i < count; i++) {
        out[i] = point_forward(matrix, &in[i]);
    }
}

// ==================== 传感器对齐实现 ====================

euler_t align_to_euler(sensor_align_e align)
{
    euler_t euler = {};

    switch (align) {
        case CW0_DEG:    break;
        case CW90_DEG:   euler.yaw = -MATH_PI_2; break;
        case CW180_DEG:  euler.yaw = -MATH_PI; break;
        case CW270_DEG:  euler.yaw = -(MATH_PI + MATH_PI_2); break;
        case CW0_FLIP:   euler.roll = MATH_PI; break;
        case CW90_FLIP:  euler.yaw = -MATH_PI_2; euler.roll = MATH_PI; break;
        case CW180_FLIP: euler.yaw = -MATH_PI; euler.roll = MATH_PI; break;
        case CW270_FLIP: euler.yaw = -(MATH_PI + MATH_PI_2); euler.roll = MATH_PI; break;
        default: break;
    }

    return euler;
}

void set_sensor_align(sensor_align_t* align, float roll, float pitch, float yaw)
{
    if (!align) return;

    align->roll = roll;
    align->pitch = pitch;
    align->yaw = yaw;
}

rot_mat_t align_to_mat(const sensor_align_t* align)
{
    rot_mat_t matrix = {};

    if (!align) return matrix;

    euler_t angles;
    angles.roll = MathDegToRad(align->roll);
    angles.pitch = MathDegToRad(align->pitch);
    angles.yaw = MathDegToRad(align->yaw);

    return euler_to_mat(&angles);
}

rot_mat_t combine_mats(const rot_mat_t* r1, const rot_mat_t* r2)
{
    rot_mat_t result = {};

    if (!r1 || !r2) return result;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.m[i][j] = r1->m[i][0] * r2->m[0][j] +
                           r1->m[i][1] * r2->m[1][j] +
                           r1->m[i][2] * r2->m[2][j];
        }
    }

    return result;
}

bool is_valid_mat(const rot_mat_t* matrix)
{
    if (!matrix) return false;

    // 检查正交性
    float c1 = matrix->m[0][0] * matrix->m[0][0] + matrix->m[1][0] * matrix->m[1][0] + matrix->m[2][0] * matrix->m[2][0];
    float c2 = matrix->m[0][1] * matrix->m[0][1] + matrix->m[1][1] * matrix->m[1][1] + matrix->m[2][1] * matrix->m[2][1];
    float c3 = matrix->m[0][2] * matrix->m[0][2] + matrix->m[1][2] * matrix->m[1][2] + matrix->m[2][2] * matrix->m[2][2];

    float d12 = matrix->m[0][0] * matrix->m[0][1] + matrix->m[1][0] * matrix->m[1][1] + matrix->m[2][0] * matrix->m[2][1];
    float d13 = matrix->m[0][0] * matrix->m[0][2] + matrix->m[1][0] * matrix->m[1][2] + matrix->m[2][0] * matrix->m[2][2];
    float d23 = matrix->m[0][1] * matrix->m[0][2] + matrix->m[1][1] * matrix->m[1][2] + matrix->m[2][1] * matrix->m[2][2];

    float det = matrix->m[0][0] * (matrix->m[1][1] * matrix->m[2][2] - matrix->m[1][2] * matrix->m[2][1])
              - matrix->m[0][1] * (matrix->m[1][0] * matrix->m[2][2] - matrix->m[1][2] * matrix->m[2][0])
              + matrix->m[0][2] * (matrix->m[1][0] * matrix->m[2][1] - matrix->m[1][1] * matrix->m[2][0]);

    const float eps = 1e-4f;

    return (MathAbs(c1 - 1.0f) < eps) && (MathAbs(c2 - 1.0f) < eps) && (MathAbs(c3 - 1.0f) < eps) &&
           (MathAbs(d12) < eps) && (MathAbs(d13) < eps) && (MathAbs(d23) < eps) &&
           (MathAbs(det - 1.0f) < eps);
}
