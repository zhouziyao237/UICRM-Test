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

#pragma once

#include <stdint.h>

// PT1低通滤波器结构
typedef struct {
    float state;
    float alpha;
} PT1Filter_t;

// 二阶滤波器结构
typedef struct {
    float state[2];
    float alpha;
} PT2Filter_t;

// 三阶滤波器结构
typedef struct {
    float state[3];
    float alpha;
} PT3Filter_t;

// 双二阶滤波器（Biquad）结构
typedef struct {
    float b0, b1, b2, a1, a2;
    float x[3], y[3];
} biquadFilter_t;

// 滤波器类型枚举
typedef enum {
    FILTER_TYPE_PT1 = 0,
    FILTER_TYPE_PT2,
    FILTER_TYPE_PT3,
    FILTER_TYPE_BIQUAD_LPF,
    FILTER_TYPE_BIQUAD_NOTCH
} filterType_e;

// 空滤波器（直接返回输入）
float nullFilterApply(float input);

// PT1滤波器相关函数
float PT1FilterGain(float cutoffFreq, float sampleTime);
void PT1FilterInit(PT1Filter_t* filter, float alpha);
float PT1FilterUpdate(PT1Filter_t* filter, float input);
void PT1FilterUpdateCutoff(PT1Filter_t *filter, float alpha);

// PT2滤波器相关函数
float PT2FilterGain(float cutoffFreq, float sampleTime);
void PT2FilterInit(PT2Filter_t* filter, float alpha);
float PT2FilterUpdate(PT2Filter_t* filter, float input);

// PT3滤波器相关函数
float PT3FilterGain(float cutoffFreq, float sampleTime);
void PT3FilterInit(PT3Filter_t* filter, float alpha);
float PT3FilterUpdate(PT3Filter_t* filter, float input);

// 双二阶滤波器相关函数
void biquadFilterInitLPF(biquadFilter_t* filter, float cutoffFreq, float sampleFreq);
void biquadFilterInitNotch(biquadFilter_t* filter, float centerFreq, float cutoffFreq,
                           float sampleFreq);
float biquadFilterUpdate(biquadFilter_t* filter, float input);

// 移动平均滤波器
typedef struct {
    float* buffer;
    uint16_t bufferSize;
    uint16_t index;
    float sum;
    uint8_t initialized;
} movingAverageFilter_t;

void movingAverageInit(movingAverageFilter_t* filter, float* buffer, uint16_t size);
float movingAverageUpdate(movingAverageFilter_t* filter, float input);

// 限幅滤波器
typedef struct {
    float lastOutput;
    float rateLimit;
} rateLimiter_t;

void rateLimiterInit(rateLimiter_t* limiter, float rateLimit);
float rateLimiterUpdate(rateLimiter_t* limiter, float input);
