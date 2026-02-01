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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bsp_ccram.h"  // 包含CCRAM定义
#include <math.h>
#include <cfloat>
#include <cmath>
#include <cstdio>

/* ==================== 硬件平台检测 ==================== */

// 自动检测STM32系列（如果bsp_ccram.h没有定义的话）
#ifndef MATH_PLATFORM_STM32F4
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || defined(STM32F439xx) || \
    defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
    #define MATH_PLATFORM_STM32F4
#endif
#endif

#ifndef MATH_PLATFORM_STM32F7
#if defined(STM32F767xx) || defined(STM32F769xx) || defined(STM32F777xx) || defined(STM32F779xx)
    #define MATH_PLATFORM_STM32F7
#endif
#endif

#ifndef MATH_PLATFORM_STM32H7
#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H753xx) || defined(STM32H7B0xx)
    #define MATH_PLATFORM_STM32H7
#endif
#endif

/* ==================== 库配置 ==================== */

// 启用CMSIS-DSP库支持
#ifndef MATH_USE_CMSIS_DSP
    #if defined(MATH_PLATFORM_STM32F4) || defined(MATH_PLATFORM_STM32F7) || defined(MATH_PLATFORM_STM32H7)
        #define MATH_USE_CMSIS_DSP 1
    #else
        #define MATH_USE_CMSIS_DSP 0
    #endif
#endif

// 启用硬件FPU
#ifndef MATH_USE_HARDWARE_FPU
    #if defined(MATH_PLATFORM_STM32F4) || defined(MATH_PLATFORM_STM32F7) || defined(MATH_PLATFORM_STM32H7)
        #define MATH_USE_HARDWARE_FPU 1
    #else
        #define MATH_USE_HARDWARE_FPU 0
    #endif
#endif

// 使用快速平方根倒数算法
#ifndef MATH_USE_FAST_INV_SQRT
    #define MATH_USE_FAST_INV_SQRT 1
#endif

// 使用快速三角函数近似
#ifndef MATH_USE_FAST_TRIG
    #define MATH_USE_FAST_TRIG 1
#endif

// 启用CCRAM优化
#ifndef MATH_USE_CCRAM_OPTIMIZATION
    #define MATH_USE_CCRAM_OPTIMIZATION 1
#endif

// 浮点精度阈值
#ifndef MATH_EPSILON
    #define MATH_EPSILON 1.19209290e-7f
#endif

/* ==================== CMSIS-DSP集成 ==================== */

#if MATH_USE_CMSIS_DSP
    // CMSIS-DSP库头文件
    #include "arm_math.h"
#endif

/* ==================== 数学常量 ==================== */

#define MATH_PI             3.14159265358979323846f
#define MATH_PI_2           1.57079632679489661923f
#define MATH_PI_4           0.78539816339744830962f
#define MATH_2PI            6.28318530717958647692f
#define MATH_INV_PI         0.31830988618379067154f
#define MATH_INV_2PI        0.15915494309189533577f
#define MATH_SQRT_2         1.41421356237309504880f
#define MATH_INV_SQRT_2     0.70710678118654752440f
#define MATH_DEG2RAD        0.01745329251994329577f
#define MATH_RAD2DEG        57.2957795130823208768f
#define MATH_E              2.71828182845904523536f
#define MATH_LN2            0.69314718055994530942f
#define MATH_INV_LN2        1.44269504088896340736f

/* ==================== 对齐宏定义（用于结构体） ==================== */

#if defined(__GNUC__) || defined(__clang__)
    #define ALIGNED_STRUCT(align_value) struct __attribute__((aligned(align_value)))
#else
    #define ALIGNED_STRUCT(align_value) struct
#endif

/* ==================== CCRAM优化的数据结构类型定义 ==================== */

// 注意：这些是类型定义，不是变量！变量定义在.cpp文件中

// 优化版2D向量类型（对齐但无section属性）
typedef ALIGNED_STRUCT(4) {
    union {
        struct {
            float x;
            float y;
        };
        float data[2];
    };
} Vec2f_opt;

// 优化版3D向量类型（对齐但无section属性）
typedef ALIGNED_STRUCT(4) {
    union {
        struct {
            float x;
            float y;
            float z;
        };
        float data[3];
    };
} Vec3f_opt;

// 优化版矩阵类型（对齐但无section属性）
typedef ALIGNED_STRUCT(4) {
    union {
        struct {
            float m00, m01, m02;
            float m10, m11, m12;
            float m20, m21, m22;
        };
        float data[9];
    };
} Mat3f_opt;

/* ==================== 基础数据结构类型定义 ==================== */

// 二维向量
typedef struct {
    float x;
    float y;
} Vec2f;

// 三维向量
typedef struct {
    float x;
    float y;
    float z;
} Vec3f;

// 向量创建宏
#define VEC2F(x, y) ((Vec2f){x, y})
#define VEC3F(x, y, z) ((Vec3f){x, y, z})

/* ==================== 矩阵类型定义 ==================== */

// 2x2矩阵
typedef struct {
    float m00, m01;
    float m10, m11;
} Mat2f;

// 3x3矩阵
typedef struct {
    float m00, m01, m02;
    float m10, m11, m12;
    float m20, m21, m22;
} Mat3f;

// 4x4矩阵
typedef struct {
    float m00, m01, m02, m03;
    float m10, m11, m12, m13;
    float m20, m21, m22, m23;
    float m30, m31, m32, m33;
} Mat4f;

/* ==================== 滤波器结构体类型定义 ==================== */

typedef struct {
    float alpha;        // 滤波系数 (0-1)
    float prevValue;    // 上一个输出值
    bool initialized;   // 是否已初始化
} FirstOrderFilter;

typedef struct {
    float a0, a1, a2;   // 分母系数
    float b0, b1, b2;   // 分子系数
    float x1, x2;       // 输入历史
    float y1, y2;       // 输出历史
} SecondOrderFilter;

typedef struct {
    float* buffer;      // 数据缓冲区
    uint32_t size;      // 缓冲区大小
    uint32_t index;     // 当前索引
    float sum;          // 当前和
    bool full;          // 缓冲区是否已满
} MovingAverageFilter;

/* ==================== 几何结构体类型定义 ==================== */

typedef struct {
    Vec2f center;
    float radius;
} Circlef;

typedef struct {
    Vec2f start;
    Vec2f end;
} LineSegmentf;

typedef struct {
    Vec2f min;
    Vec2f max;
} Rectanglef;

typedef struct {
    bool hasSolution;
    Vec2f point1;
    Vec2f point2;
} CircleIntersectionResult;

/* ==================== 性能监控结构体类型定义 ==================== */

typedef struct {
    uint32_t sin_calls;
    uint32_t cos_calls;
    uint32_t sqrt_calls;
    uint32_t atan2_calls;
    uint32_t total_operations;
    float avg_time_us;
} MathPerformanceStats;

/* ==================== 库初始化和配置函数声明 ==================== */

bool MathInit(MathPerformanceMode perf_mode);
const char* MathGetVersion(void);
const char* MathGetConfigSummary(void);
MathPerformanceMode MathGetCurrentMode(void);

/* ==================== 性能监控函数声明 ==================== */

MathPerformanceStats MathGetPerformanceStats(void);
void MathResetPerformanceStats(void);

/* ==================== 基础数学运算（CCRAM优化版）函数声明 ==================== */

float MathSqrtFast(float x);
float MathInvSqrtFast(float x);

#if MATH_USE_FAST_TRIG
float MathSinFast(float x);
float MathCosFast(float x);
#endif

Vec2f_opt Vec2fAddFast(Vec2f_opt a, Vec2f_opt b);
float Vec2fDotFast(Vec2f_opt a, Vec2f_opt b);
float Vec2fLengthFast(Vec2f_opt v);
Vec2f_opt Vec2fNormalizeFast(Vec2f_opt v);

void MathBatchSin(float* input, float* output, uint32_t count);
void MathBatchCos(float* input, float* output, uint32_t count);
void MathBatchSqrt(float* input, float* output, uint32_t count);

/* ==================== 基础数学运算（FPU加速）函数声明 ==================== */

float MathSqrt(float x);
float MathSqrtf(float x);
float MathInvSqrt(float x);
float MathExp(float x);
float MathLog(float x);
float MathLog2(float x);
float MathLog10(float x);
float MathPow(float base, float exponent);
float MathClamp(float x, float min, float max);
float MathLerp(float a, float b, float t);
float MathSmoothStep(float edge0, float edge1, float x);
float MathSign(float x);
float MathAbs(float x);
float MathMin(float a, float b);
float MathMax(float a, float b);
float MathSafeDiv(float a, float b, float defaultValue);

/* ==================== 三角函数（FPU加速）函数声明 ==================== */

float MathSin(float x);
float MathCos(float x);
float MathTan(float x);
float MathAsin(float x);
float MathAcos(float x);
float MathAtan(float x);
float MathAtan2(float y, float x);

#if MATH_USE_FAST_TRIG
float MathFastSin(float x);
float MathFastCos(float x);
#endif

/* ==================== 角度和弧度转换函数声明 ==================== */

float MathDegToRad(float deg);
float MathRadToDeg(float rad);
float MathNormalizeAngle360(float angle);
float MathNormalizeAngle180(float angle);
float MathNormalizeRad2Pi(float rad);
float MathNormalizeRadPi(float rad);
float MathAngleDiff(float a, float b);

/* ==================== 向量运算函数声明 ==================== */

Vec2f Vec2fAdd(Vec2f a, Vec2f b);
Vec2f Vec2fSub(Vec2f a, Vec2f b);
Vec2f Vec2fMul(Vec2f v, float scalar);
Vec2f Vec2fDiv(Vec2f v, float scalar);
float Vec2fDot(Vec2f a, Vec2f b);
float Vec2fCross(Vec2f a, Vec2f b);
float Vec2fLength(Vec2f v);
float Vec2fLengthSq(Vec2f v);
Vec2f Vec2fNormalize(Vec2f v);
Vec2f Vec2fRotate(Vec2f v, float angle);
float Vec2fAngle(Vec2f v);
float Vec2fAngleBetween(Vec2f a, Vec2f b);
float Vec2fDistance(Vec2f a, Vec2f b);
float Vec2fDistanceSq(Vec2f a, Vec2f b);
Vec2f Vec2fLerp(Vec2f a, Vec2f b, float t);
Vec2f Vec2fProject(Vec2f v, Vec2f onto);
Vec2f Vec2fReflect(Vec2f v, Vec2f normal);
bool Vec2fIsZero(Vec2f v);
bool Vec2fEqual(Vec2f a, Vec2f b);

Vec3f Vec3fAdd(Vec3f a, Vec3f b);
Vec3f Vec3fSub(Vec3f a, Vec3f b);
Vec3f Vec3fMul(Vec3f v, float scalar);
Vec3f Vec3fDiv(Vec3f v, float scalar);
float Vec3fDot(Vec3f a, Vec3f b);
Vec3f Vec3fCross(Vec3f a, Vec3f b);
float Vec3fLength(Vec3f v);
float Vec3fLengthSq(Vec3f v);
Vec3f Vec3fNormalize(Vec3f v);
float Vec3fDistance(Vec3f a, Vec3f b);
float Vec3fDistanceSq(Vec3f a, Vec3f b);
Vec3f Vec3fLerp(Vec3f a, Vec3f b, float t);

/* ==================== 矩阵运算函数声明 ==================== */

Mat2f Mat2fIdentity(void);
Mat2f Mat2fRotation(float angle);
Mat2f Mat2fScale(float sx, float sy);
Mat2f Mat2fMul(Mat2f a, Mat2f b);
Vec2f Mat2fMulVec(Mat2f m, Vec2f v);
float Mat2fDeterminant(Mat2f m);
Mat2f Mat2fInverse(Mat2f m);
Mat2f Mat2fTranspose(Mat2f m);

Mat3f Mat3fIdentity(void);
Mat3f Mat3fRotationX(float angle);
Mat3f Mat3fRotationY(float angle);
Mat3f Mat3fRotationZ(float angle);
Mat3f Mat3fMul(Mat3f a, Mat3f b);
Vec3f Mat3fMulVec(Mat3f m, Vec3f v);
float Mat3fDeterminant(Mat3f m);
Mat3f Mat3fInverse(Mat3f m);

#if MATH_USE_CMSIS_DSP
void Mat3fToArmMatrix(Mat3f* src, arm_matrix_instance_f32* dst);
void ArmMatrixToMat3f(arm_matrix_instance_f32* src, Mat3f* dst);
#endif

/* ==================== 滤波器函数声明 ==================== */

FirstOrderFilter FirstOrderFilterCreate(float alpha, float initialValue);
float FirstOrderFilterUpdate(FirstOrderFilter* filter, float input);

SecondOrderFilter SecondOrderFilterCreateLowpass(float cutoffFreq, float sampleFreq, float Q);
float SecondOrderFilterUpdate(SecondOrderFilter* filter, float input);

MovingAverageFilter MovingAverageFilterCreate(uint32_t size);
float MovingAverageFilterUpdate(MovingAverageFilter* filter, float input);
void MovingAverageFilterReset(MovingAverageFilter* filter);
void MovingAverageFilterDestroy(MovingAverageFilter* filter);

/* ==================== 几何运算函数声明 ==================== */

CircleIntersectionResult CircleIntersection(Circlef circle1, Circlef circle2);
Vec2f PolarToCartesian(float radius, float angle);
void CartesianToPolar(Vec2f point, float* radius, float* angle);
float PointToLineDistance(Vec2f point, LineSegmentf line);
bool PointInCircle(Vec2f point, Circlef circle);
bool PointInRectangle(Vec2f point, Rectanglef rect);
float SegmentLength(LineSegmentf segment);
float RectangleArea(Rectanglef rect);
Rectanglef RectangleExpand(Rectanglef rect, float margin);

/* ==================== 数值计算和优化函数声明 ==================== */

bool FloatEqual(float a, float b);
bool FloatNearZero(float a);
bool FloatIsValid(float x);
bool SolveLinear2x2(Mat2f a, Vec2f b, Vec2f* x);
uint8_t SolveQuadratic(float a, float b, float c, float* x1, float* x2);

/* ==================== 统计函数声明 ==================== */

float MathMean(const float* data, uint32_t count);
float MathVariance(const float* data, uint32_t count);
float MathStdDev(const float* data, uint32_t count);
float MathRMS(const float* data, uint32_t count);
void MathMinMax(const float* data, uint32_t count, float* min, float* max);

#ifdef __cplusplus
}
#endif