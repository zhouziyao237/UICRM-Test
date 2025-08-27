/*
 * Betaflight / Cleanflight Derivative Work
 * ----------------------------------------
 * Original code: Copyright (c) 2011-2025 Betaflight & Cleanflight contributors
 * Derivative work: Copyright (c) 2025 BNU-HKBU UIC RoboMaster
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

//
// Created by Administrator on 2025/8/24.
//

#include "filter.h"

#include <math.h>
#include <string.h>

#ifndef PI
#define PI 3.14159265358979f
#endif

// 空滤波器（不进行任何处理）
float nullFilterApply(float input) {
    return input;
}

// 计算PT1滤波器增益
float PT1FilterGain(float cutoffFreq, float sampleTime) {
    if (cutoffFreq <= 0.0f)
        return 1.0f;
    float rc = 1.0f / (2.0f * PI * cutoffFreq);
    return sampleTime / (rc + sampleTime);
}

// 初始化PT1滤波器
void PT1FilterInit(PT1Filter_t* filter, float alpha) {
    filter->state = 0.0f;
    filter->alpha = alpha;
}

// 更新PT1滤波器
float PT1FilterUpdate(PT1Filter_t* filter, float input) {
    filter->state += filter->alpha * (input - filter->state);
    return filter->state;
}

void PT1FilterUpdateCutoff(PT1Filter_t *filter, float alpha)
{
    filter->alpha = alpha;
}

// 计算PT2滤波器增益
float PT2FilterGain(float cutoffFreq, float sampleTime) {
    if (cutoffFreq <= 0.0f)
        return 1.0f;
    const float orderCorrection = 0.802f;  // 近似值 1/sqrt(sqrt(2) - 1)
    float rc = 1.0f / (2.0f * orderCorrection * PI * cutoffFreq);
    return sampleTime / (rc + sampleTime);
}

// 初始化PT2滤波器
void PT2FilterInit(PT2Filter_t* filter, float alpha) {
    filter->state[0] = 0.0f;
    filter->state[1] = 0.0f;
    filter->alpha = alpha;
}

// 更新PT2滤波器
float PT2FilterUpdate(PT2Filter_t* filter, float input) {
    filter->state[0] += filter->alpha * (input - filter->state[0]);
    filter->state[1] += filter->alpha * (filter->state[0] - filter->state[1]);
    return filter->state[1];
}

// 计算PT3滤波器增益
float PT3FilterGain(float cutoffFreq, float sampleTime) {
    if (cutoffFreq <= 0.0f)
        return 1.0f;
    const float orderCorrection = 0.509f;  // 近似值 1/sqrt(pow(2, 1/3) - 1)
    float rc = 1.0f / (2.0f * orderCorrection * PI * cutoffFreq);
    return sampleTime / (rc + sampleTime);
}

// 初始化PT3滤波器
void PT3FilterInit(PT3Filter_t* filter, float alpha) {
    filter->state[0] = 0.0f;
    filter->state[1] = 0.0f;
    filter->state[2] = 0.0f;
    filter->alpha = alpha;
}

// 更新PT3滤波器
float PT3FilterUpdate(PT3Filter_t* filter, float input) {
    filter->state[0] += filter->alpha * (input - filter->state[0]);
    filter->state[1] += filter->alpha * (filter->state[0] - filter->state[1]);
    filter->state[2] += filter->alpha * (filter->state[1] - filter->state[2]);
    return filter->state[2];
}

// 计算陷波滤波器Q值
static float calculateNotchQ(float centerFreq, float cutoffFreq) {
    return (centerFreq * cutoffFreq) / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

// 初始化双二阶低通滤波器
void biquadFilterInitLPF(biquadFilter_t* filter, float cutoffFreq, float sampleFreq) {
    if (cutoffFreq <= 0.0f || cutoffFreq >= sampleFreq / 2.0f) {
        // 如果截止频率无效，则将滤波器设置为直通
        filter->b0 = 1.0f;
        filter->b1 = 0.0f;
        filter->b2 = 0.0f;
        filter->a1 = 0.0f;
        filter->a2 = 0.0f;
        return;
    }

    const float omega = 2.0f * PI * cutoffFreq / sampleFreq;
    const float alpha = sinf(omega) / (2.0f * 0.7071f);  // Q = 0.7071 (巴特沃斯响应)
    const float cos_omega = cosf(omega);

    const float a0 = 1.0f + alpha;

    filter->b0 = (1.0f - cos_omega) / (2.0f * a0);
    filter->b1 = (1.0f - cos_omega) / a0;
    filter->b2 = filter->b0;
    filter->a1 = (-2.0f * cos_omega) / a0;
    filter->a2 = (1.0f - alpha) / a0;
}

// 初始化双二阶陷波滤波器
void biquadFilterInitNotch(biquadFilter_t* filter, float centerFreq, float cutoffFreq,
                           float sampleFreq) {
    if (centerFreq <= 0.0f || centerFreq >= sampleFreq / 2.0f) {
        // 如果中心频率无效，则将滤波器设置为直通
        filter->b0 = 1.0f;
        filter->b1 = 0.0f;
        filter->b2 = 0.0f;
        filter->a1 = 0.0f;
        filter->a2 = 0.0f;
        return;
    }

    const float omega = 2.0f * PI * centerFreq / sampleFreq;
    const float Q = calculateNotchQ(centerFreq, cutoffFreq);
    const float alpha = sinf(omega) / (2.0f * Q);
    const float cos_omega = cosf(omega);

    const float a0 = 1.0f + alpha;

    filter->b0 = 1.0f / a0;
    filter->b1 = (-2.0f * cos_omega) / a0;
    filter->b2 = 1.0f / a0;
    filter->a1 = filter->b1;
    filter->a2 = (1.0f - alpha) / a0;
}

// 更新双二阶滤波器
float biquadFilterUpdate(biquadFilter_t* filter, float input) {
    // 直接形式II实现
    const float output = filter->b0 * input + filter->b1 * filter->x[1] +
                         filter->b2 * filter->x[2] - filter->a1 * filter->y[1] -
                         filter->a2 * filter->y[2];

    // 更新延迟元素
    filter->x[2] = filter->x[1];
    filter->x[1] = input;
    filter->y[2] = filter->y[1];
    filter->y[1] = output;

    return output;
}

// 初始化移动平均滤波器
void movingAverageInit(movingAverageFilter_t* filter, float* buffer, uint16_t size) {
    filter->buffer = buffer;
    filter->bufferSize = size;
    filter->index = 0;
    filter->sum = 0.0f;
    filter->initialized = 0;
    memset(buffer, 0, size * sizeof(float));
}

// 更新移动平均滤波器
float movingAverageUpdate(movingAverageFilter_t* filter, float input) {
    // 从总和中减去即将被替换的值
    filter->sum -= filter->buffer[filter->index];

    // 添加新值到缓冲区和总和
    filter->buffer[filter->index] = input;
    filter->sum += input;

    // 更新索引
    filter->index = (filter->index + 1) % filter->bufferSize;

    // 检查是否已填满缓冲区
    if (!filter->initialized && filter->index == 0) {
        filter->initialized = 1;
    }

    // 返回平均值
    uint16_t count = filter->initialized ? filter->bufferSize : filter->index;
    return (count > 0) ? filter->sum / count : 0.0f;
}

// 初始化速率限制器
void rateLimiterInit(rateLimiter_t* limiter, float rateLimit) {
    limiter->lastOutput = 0.0f;
    limiter->rateLimit = rateLimit;
}

// 更新速率限制器
float rateLimiterUpdate(rateLimiter_t* limiter, float input) {
    float diff = input - limiter->lastOutput;

    if (diff > limiter->rateLimit) {
        limiter->lastOutput += limiter->rateLimit;
    } else if (diff < -limiter->rateLimit) {
        limiter->lastOutput -= limiter->rateLimit;
    } else {
        limiter->lastOutput = input;
    }

    return limiter->lastOutput;
}
