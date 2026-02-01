/* kinematics_config.h - 运动学库配置文件 */
#pragma once

#define KinematicsVersion "Kinematics Library v2.1 (CCRAM+FreeRTOS Optimized)"

// ==================== 硬件配置 ====================
// 选择使用的平台和优化级别
#define KINEMATICS_PLATFORM_STM32F4     1  // 使用STM32F4平台
#define KINEMATICS_PLATFORM_STM32F7     0
#define KINEMATICS_PLATFORM_STM32H7     0

// ==================== CCRAM配置 ====================
#define KINEMATICS_USE_CCRAM            1  // 启用CCRAM优化
#define KINEMATICS_CCRAM_SIZE          (2 * 1024)  // 分配给运动学的CCRAM大小(2KB)

// ==================== 数学库配置 ====================
#define KINEMATICS_MATH_LEVEL          2  // 0:标准, 1:FPU, 2:FPU+CCRAM, 3:FPU+CCRAM+DSP
#define KINEMATICS_USE_FAST_MATH        1  // 启用快速数学函数
#define KINEMATICS_PRECISION_MM         4  // 精度(0.1mm)

// ==================== 缓存配置 ====================
#define KINEMATICS_CACHE_ENABLED        1  // 启用计算缓存
#define KINEMATICS_CACHE_SIZE          32  // 缓存条目数
#define KINEMATICS_GRID_ENABLED         1  // 启用预计算网格
#define KINEMATICS_GRID_SIZE_X         16  // 网格X方向尺寸
#define KINEMATICS_GRID_SIZE_Y         16  // 网格Y方向尺寸

// ==================== FreeRTOS配置 ====================
#define KINEMATICS_USE_FREERTOS         1  // 启用FreeRTOS支持
#define KINEMATICS_TASK_STACK_SIZE    256  // 运动学任务栈大小(bytes)
#define KINEMATICS_QUEUE_SIZE          10  // 消息队列大小

// ==================== 性能配置 ====================
#define KINEMATICS_ENABLE_STATS         1  // 启用性能统计
#define KINEMATICS_ENABLE_PROFILING     0  // 启用详细性能分析(仅调试)
#define KINEMATICS_MAX_COMPUTE_TIME_US 100 // 最大计算时间阈值(μs)

// ==================== 内存池配置 ====================
#define KINEMATICS_USE_MEMORY_POOL      1  // 启用内存池
#define KINEMATICS_MEMORY_POOL_SIZE   512  // 内存池大小(bytes)

// ==================== 调试配置 ====================
#define KINEMATICS_ENABLE_DEBUG         0  // 启用调试输出
#define KINEMATICS_ENABLE_ASSERT        1  // 启用断言检查

// ==================== 自动配置 ====================
// 根据平台自动调整配置
#if KINEMATICS_PLATFORM_STM32F4
    #ifndef KINEMATICS_MATH_LEVEL
        #define KINEMATICS_MATH_LEVEL 2   // F4默认使用FPU+CCRAM
    #endif
    #ifndef KINEMATICS_CCRAM_SIZE
        #define KINEMATICS_CCRAM_SIZE (2 * 1024)  // F4 CCRAM有限，分配2KB
    #endif
#endif

#if KINEMATICS_PLATFORM_STM32H7
    #ifndef KINEMATICS_MATH_LEVEL
        #define KINEMATICS_MATH_LEVEL 3   // H7默认使用最高优化
    #endif
    #ifndef KINEMATICS_CCRAM_SIZE
        #define KINEMATICS_CCRAM_SIZE (8 * 1024)  // H7 CCRAM充足，分配8KB
    #endif
#endif

// 如果不使用CCRAM，关闭相关功能
#if !KINEMATICS_USE_CCRAM
    #undef KINEMATICS_GRID_ENABLED
    #define KINEMATICS_GRID_ENABLED 0
    #undef KINEMATICS_MATH_LEVEL
    #define KINEMATICS_MATH_LEVEL 1  // 降级为FPU优化
#endif