// Copyright (c) 2023-2026. BNU-HKBU UIC RoboMaster
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

#include "bsp_math.h"
#include <string.h>
#include <stdlib.h>

/* ==================== 平台特定的头文件 ==================== */

#if defined(MATH_PLATFORM_STM32F4) || defined(MATH_PLATFORM_STM32F7) || defined(MATH_PLATFORM_STM32H7)
    #include "stm32f4xx.h"
#endif

/* ==================== 内部变量声明 ==================== */

static bool mathInitialized = false;
static MathPerformanceStats perfStats = {};
static MathPerformanceMode currentMathMode = MATH_PERF_NORMAL;

#if MATH_USE_FAST_TRIG && MATH_USE_CCRAM
    // 定义在CCRAM中的三角函数表（变量定义）
    static float fastSinTable[256] CCRAM_DATA;
    static float fastCosTable[256] CCRAM_DATA;
    static bool trigTablesInitialized = false;
#endif

/* ==================== 内部函数声明 ==================== */

static void InitFPU(void);
static void InitTrigTables(void);
static inline float FastInvSqrt(float x);
static inline float FastSinApprox(float x);
static inline float FastCosApprox(float x);

/* ==================== 库初始化和管理实现 ==================== */

bool MathInit(MathPerformanceMode perf_mode) {
    if (mathInitialized) {
        return true;
    }

    currentMathMode = perf_mode;

    // 初始化FPU
    if (MATH_USE_HARDWARE_FPU) {
        InitFPU();
    }

    // 初始化CCRAM
    #if MATH_USE_CCRAM
    if (perf_mode == MATH_PERF_NORMAL_CCRAM ||
        perf_mode == MATH_PERF_ACCELERATED_CCRAM ||
        perf_mode == MATH_PERF_AUTO) {
        CCRAM_Init();
    }
    #endif

    // 初始化三角函数表
    #if MATH_USE_FAST_TRIG
        InitTrigTables();
    #endif

    mathInitialized = true;
    return true;
}

const char* MathGetVersion(void) {
    return "Integrated Math Library v2.0 with FPU/DSP/CCRAM Support";
}

const char* MathGetConfigSummary(void) {
    static char summary[256];
    snprintf(summary, sizeof(summary),
        "Config: CMSIS-DSP=%d, Hardware FPU=%d, Fast Trig=%d, Fast InvSqrt=%d, CCRAM=%d, Epsilon=%e",
        MATH_USE_CMSIS_DSP, MATH_USE_HARDWARE_FPU,
        MATH_USE_FAST_TRIG, MATH_USE_FAST_INV_SQRT,
        MATH_USE_CCRAM, MATH_EPSILON);
    return summary;
}

MathPerformanceMode MathGetCurrentMode(void) {
    return currentMathMode;
}

/* ==================== 内部辅助函数实现 ==================== */

static void InitFPU(void) {
#if MATH_USE_HARDWARE_FPU
    #if defined(MATH_PLATFORM_STM32F4) || defined(MATH_PLATFORM_STM32F7) || defined(MATH_PLATFORM_STM32H7)
        // STM32 FPU初始化
        #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
            SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
        #endif
    #endif
#endif
}

static void InitTrigTables(void) {
#if MATH_USE_FAST_TRIG && MATH_USE_CCRAM
    if (!trigTablesInitialized) {
        for (int i = 0; i < 256; i++) {
            float angle = (float)i * MATH_2PI / 256.0f;
            fastSinTable[i] = sinf(angle);
            fastCosTable[i] = cosf(angle);
        }
        trigTablesInitialized = true;
    }
#endif
}

static inline float FastInvSqrt(float x) {
    // 快速平方根倒数算法（Quake III算法）
    float xhalf = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);
    return x;
}

static inline float FastSinApprox(float x) {
    // 归一化到 [0, 2π)
    x = MathNormalizeRad2Pi(x);

    // 5阶多项式近似（平衡速度和精度）
    float x2 = x * x;
    float x3 = x * x2;
    float x5 = x3 * x2;

    return x - x3/6.0f + x5/120.0f;
}

static inline float FastCosApprox(float x) {
    // cos(x) = sin(x + π/2)
    return FastSinApprox(x + MATH_PI_2);
}

/* ==================== 性能监控实现 ==================== */

MathPerformanceStats MathGetPerformanceStats(void) {
    return perfStats;
}

void MathResetPerformanceStats(void) {
    perfStats = (MathPerformanceStats){};
}

/* ==================== 优化的数学函数实现 ==================== */

float MathSqrtFast(float x) {
    if (x < 0.0f) return 0.0f;

    #if MATH_USE_CCRAM && MATH_USE_FAST_INV_SQRT
    if (currentMathMode == MATH_PERF_ACCELERATED_CCRAM) {
        return x * FastInvSqrt(x);
    }
    #endif

    return MathSqrt(x);
}

float MathInvSqrtFast(float x) {
    if (x <= 0.0f) return 0.0f;

    #if MATH_USE_CCRAM && MATH_USE_FAST_INV_SQRT
    if (currentMathMode == MATH_PERF_ACCELERATED_CCRAM) {
        return FastInvSqrt(x);
    }
    #endif

    return 1.0f / MathSqrt(x);
}

#if MATH_USE_FAST_TRIG
float MathSinFast(float x) {
    #if MATH_USE_CCRAM
    if (currentMathMode == MATH_PERF_ACCELERATED_CCRAM && trigTablesInitialized) {
        x = MathNormalizeRad2Pi(x);
        float idx_f = x * (256.0f / MATH_2PI);
        int idx = (int)idx_f;
        float t = idx_f - idx;

        // 线性插值
        float sin1 = fastSinTable[idx % 256];
        float sin2 = fastSinTable[(idx + 1) % 256];
        return sin1 + t * (sin2 - sin1);
    }
    #endif

    return FastSinApprox(x);
}

float MathCosFast(float x) {
    #if MATH_USE_CCRAM
    if (currentMathMode == MATH_PERF_ACCELERATED_CCRAM && trigTablesInitialized) {
        x = MathNormalizeRad2Pi(x);
        float idx_f = x * (256.0f / MATH_2PI);
        int idx = (int)idx_f;
        float t = idx_f - idx;

        // 线性插值
        float cos1 = fastCosTable[idx % 256];
        float cos2 = fastCosTable[(idx + 1) % 256];
        return cos1 + t * (cos2 - cos1);
    }
    #endif

    return FastCosApprox(x);
}
#endif

/* ==================== 基础数学运算实现 ==================== */

float MathSqrt(float x) {
    if (x < 0.0f) return 0.0f;

    #if MATH_USE_CMSIS_DSP
        float result;
        arm_sqrt_f32(x, &result);
        return result;
    #elif MATH_USE_HARDWARE_FPU
        return __builtin_sqrtf(x);
    #else
        return sqrtf(x);
    #endif
}

float MathSqrtf(float x) {
    return MathSqrt(x);
}

float MathInvSqrt(float x) {
    if (x <= 0.0f) return 0.0f;

    #if MATH_USE_FAST_INV_SQRT
        return FastInvSqrt(x);
    #else
        return 1.0f / MathSqrt(x);
    #endif
}

float MathExp(float x) {
    return expf(x);
}

float MathLog(float x) {
    return logf(x);
}

float MathLog2(float x) {
    return log2f(x);
}

float MathLog10(float x) {
    return log10f(x);
}

float MathPow(float base, float exponent) {
    return powf(base, exponent);
}

float MathClamp(float x, float min, float max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

float MathLerp(float a, float b, float t) {
    t = MathClamp(t, 0.0f, 1.0f);
    return a + (b - a) * t;
}

float MathSmoothStep(float edge0, float edge1, float x) {
    x = MathClamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

float MathSign(float x) {
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

float MathAbs(float x) {
    return fabsf(x);
}

float MathMin(float a, float b) {
    return (a < b) ? a : b;
}

float MathMax(float a, float b) {
    return (a > b) ? a : b;
}

float MathSafeDiv(float a, float b, float defaultValue) {
    if (fabsf(b) < MATH_EPSILON) {
        return defaultValue;
    }
    return a / b;
}

float MathSin(float x) {
    #if MATH_USE_CMSIS_DSP
    if (currentMathMode == MATH_PERF_ACCELERATED ||
        currentMathMode == MATH_PERF_ACCELERATED_CCRAM) {
        return arm_sin_f32(x);
    }
    #endif

    #if MATH_USE_FAST_TRIG
    if (currentMathMode == MATH_PERF_ACCELERATED ||
        currentMathMode == MATH_PERF_ACCELERATED_CCRAM) {
        return MathSinFast(x);
    }
    #endif

    return sinf(x);
}

float MathCos(float x) {
    #if MATH_USE_CMSIS_DSP
    if (currentMathMode == MATH_PERF_ACCELERATED ||
        currentMathMode == MATH_PERF_ACCELERATED_CCRAM) {
        return arm_cos_f32(x);
    }
    #endif

    #if MATH_USE_FAST_TRIG
    if (currentMathMode == MATH_PERF_ACCELERATED ||
        currentMathMode == MATH_PERF_ACCELERATED_CCRAM) {
        return MathCosFast(x);
    }
    #endif

    return cosf(x);
}

float MathTan(float x) {
#if MATH_USE_CMSIS_DSP
    // CMSIS-DSP没有tan函数，使用sin/cos
    float cosVal = arm_cos_f32(x);
    if (fabsf(cosVal) < MATH_EPSILON) return 0.0f;
    return arm_sin_f32(x) / cosVal;
#else
    return tanf(x);
#endif
}

float MathAsin(float x) {
    x = MathClamp(x, -1.0f, 1.0f);
    return asinf(x);
}

float MathAcos(float x) {
    x = MathClamp(x, -1.0f, 1.0f);
    return acosf(x);
}

float MathAtan(float x) {
    return atanf(x);
}

float MathAtan2(float y, float x) {
    return atan2f(y, x);
}

#if MATH_USE_FAST_TRIG
float MathFastSin(float x) {
    return FastSinApprox(x);
}

float MathFastCos(float x) {
    return FastCosApprox(x);
}
#endif

/* ==================== 角度和弧度转换实现 ==================== */

float MathDegToRad(float deg) {
    return deg * MATH_DEG2RAD;
}

float MathRadToDeg(float rad) {
    return rad * MATH_RAD2DEG;
}

float MathNormalizeAngle360(float angle) {
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) {
        angle += 360.0f;
    }
    return angle;
}

float MathNormalizeAngle180(float angle) {
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0.0f) {
        angle += 360.0f;
    }
    return angle - 180.0f;
}

float MathNormalizeRad2Pi(float rad) {
    rad = fmodf(rad, MATH_2PI);
    if (rad < 0.0f) {
        rad += MATH_2PI;
    }
    return rad;
}

float MathNormalizeRadPi(float rad) {
    rad = fmodf(rad + MATH_PI, MATH_2PI);
    if (rad < 0.0f) {
        rad += MATH_2PI;
    }
    return rad - MATH_PI;
}

float MathAngleDiff(float a, float b) {
    float diff = MathNormalizeAngle360(a) - MathNormalizeAngle360(b);
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

/* ==================== 向量运算实现 ==================== */

Vec2f Vec2fAdd(Vec2f a, Vec2f b) {
    Vec2f result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    return result;
}

Vec2f Vec2fSub(Vec2f a, Vec2f b) {
    Vec2f result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    return result;
}

Vec2f Vec2fMul(Vec2f v, float scalar) {
    Vec2f result;
    result.x = v.x * scalar;
    result.y = v.y * scalar;
    return result;
}

Vec2f Vec2fDiv(Vec2f v, float scalar) {
    if (fabsf(scalar) < MATH_EPSILON) {
        return VEC2F(0.0f, 0.0f);
    }
    Vec2f result;
    result.x = v.x / scalar;
    result.y = v.y / scalar;
    return result;
}

float Vec2fDot(Vec2f a, Vec2f b) {
    return a.x * b.x + a.y * b.y;
}

float Vec2fCross(Vec2f a, Vec2f b) {
    return a.x * b.y - a.y * b.x;
}

float Vec2fLength(Vec2f v) {
    return MathSqrt(v.x * v.x + v.y * v.y);
}

float Vec2fLengthSq(Vec2f v) {
    return v.x * v.x + v.y * v.y;
}

Vec2f Vec2fNormalize(Vec2f v) {
    float length = Vec2fLength(v);
    if (length < MATH_EPSILON) {
        return VEC2F(0.0f, 0.0f);
    }
    return Vec2fDiv(v, length);
}

Vec2f Vec2fRotate(Vec2f v, float angle) {
    float cosVal = MathCos(angle);
    float sinVal = MathSin(angle);

    Vec2f result;
    result.x = v.x * cosVal - v.y * sinVal;
    result.y = v.x * sinVal + v.y * cosVal;
    return result;
}

float Vec2fAngle(Vec2f v) {
    return MathAtan2(v.y, v.x);
}

float Vec2fAngleBetween(Vec2f a, Vec2f b) {
    float dotVal = Vec2fDot(a, b);
    float lenA = Vec2fLength(a);
    float lenB = Vec2fLength(b);

    if (lenA < MATH_EPSILON || lenB < MATH_EPSILON) {
        return 0.0f;
    }

    float cosVal = MathClamp(dotVal / (lenA * lenB), -1.0f, 1.0f);
    return MathAcos(cosVal);
}

float Vec2fDistance(Vec2f a, Vec2f b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return MathSqrt(dx * dx + dy * dy);
}

float Vec2fDistanceSq(Vec2f a, Vec2f b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return dx * dx + dy * dy;
}

Vec2f Vec2fLerp(Vec2f a, Vec2f b, float t) {
    t = MathClamp(t, 0.0f, 1.0f);
    Vec2f result;
    result.x = a.x + (b.x - a.x) * t;
    result.y = a.y + (b.y - a.y) * t;
    return result;
}

Vec2f Vec2fProject(Vec2f v, Vec2f onto) {
    float lenSq = Vec2fLengthSq(onto);
    if (lenSq < MATH_EPSILON) {
        return VEC2F(0.0f, 0.0f);
    }

    float dotVal = Vec2fDot(v, onto);
    float scalar = dotVal / lenSq;
    return Vec2fMul(onto, scalar);
}

Vec2f Vec2fReflect(Vec2f v, Vec2f normal) {
    Vec2f norm = Vec2fNormalize(normal);
    float dotVal = Vec2fDot(v, norm);
    return Vec2fSub(v, Vec2fMul(norm, 2.0f * dotVal));
}

bool Vec2fIsZero(Vec2f v) {
    return (fabsf(v.x) < MATH_EPSILON) && (fabsf(v.y) < MATH_EPSILON);
}

bool Vec2fEqual(Vec2f a, Vec2f b) {
    return (fabsf(a.x - b.x) < MATH_EPSILON) && (fabsf(a.y - b.y) < MATH_EPSILON);
}

Vec3f Vec3fAdd(Vec3f a, Vec3f b) {
    Vec3f result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

Vec3f Vec3fSub(Vec3f a, Vec3f b) {
    Vec3f result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

Vec3f Vec3fMul(Vec3f v, float scalar) {
    Vec3f result;
    result.x = v.x * scalar;
    result.y = v.y * scalar;
    result.z = v.z * scalar;
    return result;
}

Vec3f Vec3fDiv(Vec3f v, float scalar) {
    if (fabsf(scalar) < MATH_EPSILON) {
        return VEC3F(0.0f, 0.0f, 0.0f);
    }
    Vec3f result;
    result.x = v.x / scalar;
    result.y = v.y / scalar;
    result.z = v.z / scalar;
    return result;
}

float Vec3fDot(Vec3f a, Vec3f b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3f Vec3fCross(Vec3f a, Vec3f b) {
    Vec3f result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}

float Vec3fLength(Vec3f v) {
    return MathSqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

float Vec3fLengthSq(Vec3f v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

Vec3f Vec3fNormalize(Vec3f v) {
    float length = Vec3fLength(v);
    if (length < MATH_EPSILON) {
        return VEC3F(0.0f, 0.0f, 0.0f);
    }
    return Vec3fDiv(v, length);
}

float Vec3fDistance(Vec3f a, Vec3f b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return MathSqrt(dx * dx + dy * dy + dz * dz);
}

float Vec3fDistanceSq(Vec3f a, Vec3f b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

Vec3f Vec3fLerp(Vec3f a, Vec3f b, float t) {
    t = MathClamp(t, 0.0f, 1.0f);
    Vec3f result;
    result.x = a.x + (b.x - a.x) * t;
    result.y = a.y + (b.y - a.y) * t;
    result.z = a.z + (b.z - a.z) * t;
    return result;
}

/* ==================== 优化的向量运算实现 ==================== */

Vec2f_opt Vec2fAddFast(Vec2f_opt a, Vec2f_opt b) {
    Vec2f_opt result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    return result;
}

float Vec2fDotFast(Vec2f_opt a, Vec2f_opt b) {
    return a.x * b.x + a.y * b.y;
}

float Vec2fLengthFast(Vec2f_opt v) {
    return MathSqrtFast(v.x * v.x + v.y * v.y);
}

Vec2f_opt Vec2fNormalizeFast(Vec2f_opt v) {
    float length = Vec2fLengthFast(v);
    if (length < MATH_EPSILON) {
        Vec2f_opt result = {};
        return result;
    }
    Vec2f_opt result;
    result.x = v.x / length;
    result.y = v.y / length;
    return result;
}

/* ==================== 批量运算实现 ==================== */

void MathBatchSin(float* input, float* output, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        output[i] = MathSin(input[i]);
    }
}

void MathBatchCos(float* input, float* output, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        output[i] = MathCos(input[i]);
    }
}

void MathBatchSqrt(float* input, float* output, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        output[i] = MathSqrt(input[i]);
    }
}

/* ==================== 矩阵运算实现 ==================== */

Mat2f Mat2fIdentity(void) {
    Mat2f m = {1.0f, 0.0f, 0.0f, 1.0f};
    return m;
}

Mat2f Mat2fRotation(float angle) {
    float cosVal = MathCos(angle);
    float sinVal = MathSin(angle);

    Mat2f m = {cosVal, -sinVal, sinVal, cosVal};
    return m;
}

Mat2f Mat2fScale(float sx, float sy) {
    Mat2f m = {sx, 0.0f, 0.0f, sy};
    return m;
}

Mat2f Mat2fMul(Mat2f a, Mat2f b) {
    Mat2f result;
    result.m00 = a.m00 * b.m00 + a.m01 * b.m10;
    result.m01 = a.m00 * b.m01 + a.m01 * b.m11;
    result.m10 = a.m10 * b.m00 + a.m11 * b.m10;
    result.m11 = a.m10 * b.m01 + a.m11 * b.m11;
    return result;
}

Vec2f Mat2fMulVec(Mat2f m, Vec2f v) {
    Vec2f result;
    result.x = m.m00 * v.x + m.m01 * v.y;
    result.y = m.m10 * v.x + m.m11 * v.y;
    return result;
}

float Mat2fDeterminant(Mat2f m) {
    return m.m00 * m.m11 - m.m01 * m.m10;
}

Mat2f Mat2fInverse(Mat2f m) {
    float det = Mat2fDeterminant(m);
    Mat2f result;

    if (fabsf(det) < MATH_EPSILON) {
        return Mat2fIdentity(); // 无法求逆，返回单位矩阵
    }

    float invDet = 1.0f / det;
    result.m00 = m.m11 * invDet;
    result.m01 = -m.m01 * invDet;
    result.m10 = -m.m10 * invDet;
    result.m11 = m.m00 * invDet;

    return result;
}

Mat2f Mat2fTranspose(Mat2f m) {
    Mat2f result;
    result.m00 = m.m00;
    result.m01 = m.m10;
    result.m10 = m.m01;
    result.m11 = m.m11;
    return result;
}

Mat3f Mat3fIdentity(void) {
    Mat3f m = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };
    return m;
}

Mat3f Mat3fRotationX(float angle) {
    float cosVal = MathCos(angle);
    float sinVal = MathSin(angle);

    Mat3f m = {
        1.0f, 0.0f, 0.0f,
        0.0f, cosVal, -sinVal,
        0.0f, sinVal, cosVal
    };
    return m;
}

Mat3f Mat3fRotationY(float angle) {
    float cosVal = MathCos(angle);
    float sinVal = MathSin(angle);

    Mat3f m = {
        cosVal, 0.0f, sinVal,
        0.0f, 1.0f, 0.0f,
        -sinVal, 0.0f, cosVal
    };
    return m;
}

Mat3f Mat3fRotationZ(float angle) {
    float cosVal = MathCos(angle);
    float sinVal = MathSin(angle);

    Mat3f m = {
        cosVal, -sinVal, 0.0f,
        sinVal, cosVal, 0.0f,
        0.0f, 0.0f, 1.0f
    };
    return m;
}

Mat3f Mat3fMul(Mat3f a, Mat3f b) {
    Mat3f result;

    result.m00 = a.m00 * b.m00 + a.m01 * b.m10 + a.m02 * b.m20;
    result.m01 = a.m00 * b.m01 + a.m01 * b.m11 + a.m02 * b.m21;
    result.m02 = a.m00 * b.m02 + a.m01 * b.m12 + a.m02 * b.m22;

    result.m10 = a.m10 * b.m00 + a.m11 * b.m10 + a.m12 * b.m20;
    result.m11 = a.m10 * b.m01 + a.m11 * b.m11 + a.m12 * b.m21;
    result.m12 = a.m10 * b.m02 + a.m11 * b.m12 + a.m12 * b.m22;

    result.m20 = a.m20 * b.m00 + a.m21 * b.m10 + a.m22 * b.m20;
    result.m21 = a.m20 * b.m01 + a.m21 * b.m11 + a.m22 * b.m21;
    result.m22 = a.m20 * b.m02 + a.m21 * b.m12 + a.m22 * b.m22;

    return result;
}

Vec3f Mat3fMulVec(Mat3f m, Vec3f v) {
    Vec3f result;
    result.x = m.m00 * v.x + m.m01 * v.y + m.m02 * v.z;
    result.y = m.m10 * v.x + m.m11 * v.y + m.m12 * v.z;
    result.z = m.m20 * v.x + m.m21 * v.y + m.m22 * v.z;
    return result;
}

float Mat3fDeterminant(Mat3f m) {
    return m.m00 * (m.m11 * m.m22 - m.m12 * m.m21)
         - m.m01 * (m.m10 * m.m22 - m.m12 * m.m20)
         + m.m02 * (m.m10 * m.m21 - m.m11 * m.m20);
}

Mat3f Mat3fInverse(Mat3f m) {
    float det = Mat3fDeterminant(m);
    Mat3f result;

    if (fabsf(det) < MATH_EPSILON) {
        return Mat3fIdentity(); // 无法求逆，返回单位矩阵
    }

    float invDet = 1.0f / det;

    result.m00 = (m.m11 * m.m22 - m.m12 * m.m21) * invDet;
    result.m01 = (m.m02 * m.m21 - m.m01 * m.m22) * invDet;
    result.m02 = (m.m01 * m.m12 - m.m02 * m.m11) * invDet;

    result.m10 = (m.m12 * m.m20 - m.m10 * m.m22) * invDet;
    result.m11 = (m.m00 * m.m22 - m.m02 * m.m20) * invDet;
    result.m12 = (m.m02 * m.m10 - m.m00 * m.m12) * invDet;

    result.m20 = (m.m10 * m.m21 - m.m11 * m.m20) * invDet;
    result.m21 = (m.m01 * m.m20 - m.m00 * m.m21) * invDet;
    result.m22 = (m.m00 * m.m11 - m.m01 * m.m10) * invDet;

    return result;
}

#if MATH_USE_CMSIS_DSP

void Mat3fToArmMatrix(Mat3f* src, arm_matrix_instance_f32* dst) {
    dst->numRows = 3;
    dst->numCols = 3;
    dst->pData[0] = src->m00;
    dst->pData[1] = src->m01;
    dst->pData[2] = src->m02;
    dst->pData[3] = src->m10;
    dst->pData[4] = src->m11;
    dst->pData[5] = src->m12;
    dst->pData[6] = src->m20;
    dst->pData[7] = src->m21;
    dst->pData[8] = src->m22;
}

void ArmMatrixToMat3f(arm_matrix_instance_f32* src, Mat3f* dst) {
    if (src->numRows != 3 || src->numCols != 3) {
        // 大小不匹配，返回单位矩阵
        *dst = Mat3fIdentity();
        return;
    }

    dst->m00 = src->pData[0];
    dst->m01 = src->pData[1];
    dst->m02 = src->pData[2];
    dst->m10 = src->pData[3];
    dst->m11 = src->pData[4];
    dst->m12 = src->pData[5];
    dst->m20 = src->pData[6];
    dst->m21 = src->pData[7];
    dst->m22 = src->pData[8];
}

#endif

/* ==================== 滤波器函数实现 ==================== */

FirstOrderFilter FirstOrderFilterCreate(float alpha, float initialValue) {
    FirstOrderFilter filter;
    filter.alpha = MathClamp(alpha, 0.0f, 1.0f);
    filter.prevValue = initialValue;
    filter.initialized = true;
    return filter;
}

float FirstOrderFilterUpdate(FirstOrderFilter* filter, float input) {
    if (!filter->initialized) {
        filter->prevValue = input;
        filter->initialized = true;
        return input;
    }

    filter->prevValue = filter->alpha * input + (1.0f - filter->alpha) * filter->prevValue;
    return filter->prevValue;
}

SecondOrderFilter SecondOrderFilterCreateLowpass(float cutoffFreq, float sampleFreq, float Q) {
    SecondOrderFilter filter;

    float omega = 2.0f * MATH_PI * cutoffFreq / sampleFreq;
    float sinOmega = MathSin(omega);
    float cosOmega = MathCos(omega);
    float alpha = sinOmega / (2.0f * Q);

    float b0 = (1.0f - cosOmega) / 2.0f;
    float b1 = 1.0f - cosOmega;
    float b2 = b0;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cosOmega;
    float a2 = 1.0f - alpha;

    filter.b0 = b0 / a0;
    filter.b1 = b1 / a0;
    filter.b2 = b2 / a0;
    filter.a1 = a1 / a0;
    filter.a2 = a2 / a0;

    filter.x1 = filter.x2 = 0.0f;
    filter.y1 = filter.y2 = 0.0f;

    return filter;
}

float SecondOrderFilterUpdate(SecondOrderFilter* filter, float input) {
    float output = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

MovingAverageFilter MovingAverageFilterCreate(uint32_t size) {
    // 在CCRAM中分配内存
    #if MATH_USE_CCRAM
    MovingAverageFilter filter;
    filter.buffer = (float*)CCRAM_Malloc(size * sizeof(float));
    filter.size = size;
    filter.index = 0;
    filter.sum = 0.0f;
    filter.full = false;

    if (filter.buffer) {
        memset(filter.buffer, 0, size * sizeof(float));
    }

    return filter;
    #else
    // 如果没有CCRAM，使用普通内存
    MovingAverageFilter filter;
    filter.buffer = (float*)malloc(size * sizeof(float));
    filter.size = size;
    filter.index = 0;
    filter.sum = 0.0f;
    filter.full = false;

    if (filter.buffer) {
        memset(filter.buffer, 0, size * sizeof(float));
    }

    return filter;
    #endif
}

float MovingAverageFilterUpdate(MovingAverageFilter* filter, float input) {
    if (!filter->buffer) {
        return input;
    }

    // 减去即将被替换的旧值
    filter->sum -= filter->buffer[filter->index];

    // 添加新值
    filter->buffer[filter->index] = input;
    filter->sum += input;

    // 更新索引
    filter->index = (filter->index + 1) % filter->size;

    if (filter->index == 0) {
        filter->full = true;
    }

    // 计算平均值
    uint32_t count = filter->full ? filter->size : filter->index;
    return filter->sum / count;
}

void MovingAverageFilterReset(MovingAverageFilter* filter) {
    if (filter->buffer) {
        #if MATH_USE_CCRAM
        CCRAM_Memset(filter->buffer, 0, filter->size * sizeof(float));
        #else
        memset(filter->buffer, 0, filter->size * sizeof(float));
        #endif
    }
    filter->index = 0;
    filter->sum = 0.0f;
    filter->full = false;
}

void MovingAverageFilterDestroy(MovingAverageFilter* filter) {
    if (filter->buffer) {
        #if MATH_USE_CCRAM
        CCRAM_Free(filter->buffer);
        #else
        free(filter->buffer);
        #endif
        filter->buffer = NULL;
    }
    filter->size = 0;
    filter->index = 0;
    filter->sum = 0.0f;
    filter->full = false;
}

/* ==================== 几何运算实现 ==================== */

CircleIntersectionResult CircleIntersection(Circlef circle1, Circlef circle2) {
    CircleIntersectionResult result;
    result.hasSolution = false;

    Vec2f dVec = Vec2fSub(circle2.center, circle1.center);
    float distanceSq = Vec2fLengthSq(dVec);
    float distance = MathSqrt(distanceSq);

    // 检查是否有交点
    if (distance > circle1.radius + circle2.radius ||
        distance < fabsf(circle1.radius - circle2.radius)) {
        return result;
    }

    // 计算交点
    float a = (circle1.radius * circle1.radius -
               circle2.radius * circle2.radius +
               distanceSq) / (2.0f * distance);

    float hSq = circle1.radius * circle1.radius - a * a;

    if (hSq < 0.0f) {
        return result;
    }

    float h = MathSqrt(hSq);

    // 中点
    Vec2f midpoint = Vec2fAdd(circle1.center,
                             Vec2fMul(Vec2fNormalize(dVec), a));

    // 垂直方向向量
    Vec2f perp = VEC2F(-dVec.y, dVec.x);
    Vec2f perpUnit = Vec2fNormalize(perp);

    // 两个交点
    result.point1 = Vec2fAdd(midpoint, Vec2fMul(perpUnit, h));
    result.point2 = Vec2fSub(midpoint, Vec2fMul(perpUnit, h));
    result.hasSolution = true;

    return result;
}

Vec2f PolarToCartesian(float radius, float angle) {
    Vec2f result;
    result.x = radius * MathCos(angle);
    result.y = radius * MathSin(angle);
    return result;
}

void CartesianToPolar(Vec2f point, float* radius, float* angle) {
    if (radius) {
        *radius = Vec2fLength(point);
    }
    if (angle) {
        *angle = MathAtan2(point.y, point.x);
    }
}

float PointToLineDistance(Vec2f point, LineSegmentf line) {
    Vec2f v = Vec2fSub(line.end, line.start);
    Vec2f w = Vec2fSub(point, line.start);

    float c1 = Vec2fDot(w, v);
    if (c1 <= 0.0f) {
        return Vec2fDistance(point, line.start);
    }

    float c2 = Vec2fDot(v, v);
    if (c2 <= c1) {
        return Vec2fDistance(point, line.end);
    }

    float b = c1 / c2;
    Vec2f pb = Vec2fAdd(line.start, Vec2fMul(v, b));
    return Vec2fDistance(point, pb);
}

bool PointInCircle(Vec2f point, Circlef circle) {
    float dx = point.x - circle.center.x;
    float dy = point.y - circle.center.y;
    float distanceSq = dx * dx + dy * dy;
    return distanceSq <= circle.radius * circle.radius;
}

bool PointInRectangle(Vec2f point, Rectanglef rect) {
    return (point.x >= rect.min.x && point.x <= rect.max.x &&
            point.y >= rect.min.y && point.y <= rect.max.y);
}

float SegmentLength(LineSegmentf segment) {
    return Vec2fDistance(segment.start, segment.end);
}

float RectangleArea(Rectanglef rect) {
    float width = rect.max.x - rect.min.x;
    float height = rect.max.y - rect.min.y;
    return width * height;
}

Rectanglef RectangleExpand(Rectanglef rect, float margin) {
    Rectanglef result;
    result.min.x = rect.min.x - margin;
    result.min.y = rect.min.y - margin;
    result.max.x = rect.max.x + margin;
    result.max.y = rect.max.y + margin;
    return result;
}

/* ==================== 数值计算和优化实现 ==================== */

bool FloatEqual(float a, float b) {
    return MathAbs(a - b) < MATH_EPSILON;
}

bool FloatNearZero(float a) {
    return MathAbs(a) < MATH_EPSILON;
}

bool FloatIsValid(float x) {
    // 检查是否为NaN或无穷大
    return (x == x) && (x <= FLT_MAX) && (x >= -FLT_MAX);
}

bool SolveLinear2x2(Mat2f a, Vec2f b, Vec2f* x) {
    float det = Mat2fDeterminant(a);

    if (fabsf(det) < MATH_EPSILON) {
        return false;
    }

    float invDet = 1.0f / det;

    if (x) {
        x->x = (a.m11 * b.x - a.m01 * b.y) * invDet;
        x->y = (a.m00 * b.y - a.m10 * b.x) * invDet;
    }

    return true;
}

uint8_t SolveQuadratic(float a, float b, float c, float* x1, float* x2) {
    if (fabsf(a) < MATH_EPSILON) {
        // 退化为线性方程
        if (fabsf(b) < MATH_EPSILON) {
            return 0; // 无解
        }

        if (x1) *x1 = -c / b;
        return 1;
    }

    float discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f) {
        return 0; // 无实数解
    }

    if (fabsf(discriminant) < MATH_EPSILON) {
        if (x1) *x1 = -b / (2.0f * a);
        return 1;
    }

    float sqrtDiscriminant = MathSqrt(discriminant);
    float denominator = 2.0f * a;

    if (x1) *x1 = (-b - sqrtDiscriminant) / denominator;
    if (x2) *x2 = (-b + sqrtDiscriminant) / denominator;

    return 2;
}

/* ==================== 统计函数实现 ==================== */

float MathMean(const float* data, uint32_t count) {
    if (count == 0) return 0.0f;

    float sum = 0.0f;
    for (uint32_t i = 0; i < count; i++) {
        sum += data[i];
    }

    return sum / count;
}

float MathVariance(const float* data, uint32_t count) {
    if (count <= 1) return 0.0f;

    float mean = MathMean(data, count);
    float sum = 0.0f;

    for (uint32_t i = 0; i < count; i++) {
        float diff = data[i] - mean;
        sum += diff * diff;
    }

    return sum / (count - 1);
}

float MathStdDev(const float* data, uint32_t count) {
    return MathSqrt(MathVariance(data, count));
}

float MathRMS(const float* data, uint32_t count) {
    if (count == 0) return 0.0f;

    float sum = 0.0f;
    for (uint32_t i = 0; i < count; i++) {
        sum += data[i] * data[i];
    }

    return MathSqrt(sum / count);
}

void MathMinMax(const float* data, uint32_t count, float* min, float* max) {
    if (count == 0) {
        if (min) *min = 0.0f;
        if (max) *max = 0.0f;
        return;
    }

    float minVal = data[0];
    float maxVal = data[0];

    for (uint32_t i = 1; i < count; i++) {
        if (data[i] < minVal) minVal = data[i];
        if (data[i] > maxVal) maxVal = data[i];
    }

    if (min) *min = minVal;
    if (max) *max = maxVal;
}