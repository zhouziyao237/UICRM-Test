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

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== CCRAM平台检测 ==================== */

// 检查是否支持CCRAM
#ifndef MATH_USE_CCRAM
    #if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
        defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || defined(STM32F439xx) || \
        defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
        #define MATH_PLATFORM_STM32F4
        #define MATH_USE_CCRAM 1
    #elif defined(STM32F767xx) || defined(STM32F769xx) || defined(STM32F777xx) || defined(STM32F779xx)
        #define MATH_PLATFORM_STM32F7
        #define MATH_USE_CCRAM 1
    #elif defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H753xx) || defined(STM32H7B0xx)
        #define MATH_PLATFORM_STM32H7
        #define MATH_USE_CCRAM 1
    #else
        #define MATH_USE_CCRAM 0
    #endif
#endif

/* ==================== 性能模式定义 ==================== */

// 性能模式枚举
typedef enum {
    MATH_PERF_NORMAL = 0,        // 普通模式
    MATH_PERF_NORMAL_CCRAM,      // 普通+CCRAM模式
    MATH_PERF_ACCELERATED,       // 数学加速模式
    MATH_PERF_ACCELERATED_CCRAM, // 数学加速+CCRAM模式
    MATH_PERF_AUTO               // 自动选择
} MathPerformanceMode;

/* ==================== 数学库模式定义 ==================== */

// 库运行模式
typedef enum {
    MATH_MODE_AUTO = 0,          // 自动检测
    MATH_MODE_ACCURACY,          // 高精度模式
    MATH_MODE_PERFORMANCE,       // 高性能模式
    MATH_MODE_MINIMAL            // 最小内存模式
} MathLibraryMode;

/* ==================== CCRAM内存区域定义 ==================== */

#if MATH_USE_CCRAM
    // STM32F4 CCRAM地址和大小
    #ifdef MATH_PLATFORM_STM32F4
        #define CCRAM_BASE_ADDR   0x10000000U
        #define CCRAM_SIZE        (64 * 1024)  // 64KB
    #elif defined(MATH_PLATFORM_STM32F7)
        #define CCRAM_BASE_ADDR   0x20000000U  // D1域SRAM
        #define CCRAM_SIZE        (128 * 1024)
    #elif defined(MATH_PLATFORM_STM32H7)
        #define CCRAM_BASE_ADDR   0x24000000U  // AXI SRAM
        #define CCRAM_SIZE        (512 * 1024)
    #else
        #define CCRAM_BASE_ADDR   0x10000000U
        #define CCRAM_SIZE        (64 * 1024)
    #endif

    // CCRAM变量属性宏 - 只能用于变量声明
    #define CCRAM_DATA           __attribute__((section(".ccmram")))
    #define CCRAM_BSS            __attribute__((section(".ccmram_bss")))

    // 对齐宏
    #define CCRAM_ALIGN(n)       __attribute__((aligned(n)))
    #define CCRAM_CACHE_ALIGN    CCRAM_ALIGN(32)

    // 强制内联优化
    #define MATH_FORCE_INLINE    __attribute__((always_inline)) inline

    // 对齐宏（用于类型定义）
    #define MATH_ALIGN(n)        __attribute__((aligned(n)))
    #define MATH_CACHE_ALIGN     MATH_ALIGN(32)

#else
    // 非CCRAM平台
    #define CCRAM_DATA
    #define CCRAM_BSS
    #define CCRAM_ALIGN(n)
    #define CCRAM_CACHE_ALIGN
    #define MATH_FORCE_INLINE    inline
    #define MATH_ALIGN(n)
    #define MATH_CACHE_ALIGN
#endif

/* ==================== CCRAM内存管理API ==================== */

#if MATH_USE_CCRAM
/**
 * @brief 初始化CCRAM管理器
 * @return 是否成功
 */
bool CCRAM_Init(void);

/**
 * @brief 从CCRAM分配内存
 * @param size 分配大小
 * @return 分配的内存指针，失败返回NULL
 */
void* CCRAM_Malloc(size_t size);

/**
 * @brief 从CCRAM分配并清零内存
 * @param num 元素个数
 * @param size 元素大小
 * @return 分配的内存指针
 */
void* CCRAM_Calloc(size_t num, size_t size);

/**
 * @brief 释放CCRAM内存
 * @param ptr 内存指针
 */
void CCRAM_Free(void* ptr);

/**
 * @brief 获取CCRAM使用统计
 * @param used 已使用字节数
 * @param free 空闲字节数
 * @param fragments 碎片数量
 */
void CCRAM_GetStats(size_t* used, size_t* free, uint32_t* fragments);

/**
 * @brief 检查指针是否在CCRAM区域
 * @param ptr 要检查的指针
 * @return 是否在CCRAM中
 */
bool CCRAM_IsInCCRAM(const void* ptr);

/**
 * @brief 设置性能模式
 * @param mode 性能模式
 */
void MathSetPerformanceMode(MathPerformanceMode mode);

/**
 * @brief 获取当前性能模式
 * @return 当前性能模式
 */
MathPerformanceMode MathGetPerformanceMode(void);

#else
// 非CCRAM平台的空实现
static inline bool CCRAM_Init(void) { return true; }
static inline void* CCRAM_Malloc(size_t size) {
    (void)size;
    return NULL;
}
static inline void* CCRAM_Calloc(size_t num, size_t size) {
    (void)num; (void)size;
    return NULL;
}
static inline void CCRAM_Free(void* ptr) {
    (void)ptr;
}
static inline void CCRAM_GetStats(size_t* used, size_t* free, uint32_t* fragments) {
    if (used) *used = 0;
    if (free) *free = 0;
    if (fragments) *fragments = 0;
}
static inline bool CCRAM_IsInCCRAM(const void* ptr) {
    (void)ptr;
    return false;
}
static inline void MathSetPerformanceMode(MathPerformanceMode mode) {
    (void)mode;
}
static inline MathPerformanceMode MathGetPerformanceMode(void) {
    return MATH_PERF_NORMAL;
}
#endif

#ifdef __cplusplus
}
#endif

// 用于标记类成员应该在CCRAM中的宏
#if MATH_USE_CCRAM
#define CCRAM_MEMBER    CCRAM_DATA
#else
#define CCRAM_MEMBER
#endif