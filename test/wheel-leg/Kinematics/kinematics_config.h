/* kinematics_config.h - 运动学库配置文件 */
#pragma once

#define KIN_VERSION "Kinematics Library v2.2 (Layered Architecture)"

// ==================== 硬件平台 ====================
#define KIN_PLATFORM_AUTO    1  // 自动检测平台
#define KIN_PLATFORM_STM32F4 0
#define KIN_PLATFORM_STM32F7 0
#define KIN_PLATFORM_STM32H7 0

// ==================== 功能开关 ====================
#define KIN_USE_CACHE   1  // 启用缓存
#define KIN_USE_FAST    1  // 启用快速数学
#define KIN_USE_RTOS    0  // 禁用RTOS支持（减少内存占用）
#define KIN_USE_CCRAM   1  // 启用CCRAM
#define KIN_USE_GRID    0  // 启用网格（已禁用以节省RAM）
#define KIN_USE_STATS   1  // 启用统计

// ==================== 性能配置 ====================
#define KIN_CACHE_SIZE  8    // 缓存条目数（进一步减少以节省RAM）
#define KIN_GRID_X      8    // 网格X尺寸（即使启用了也较小）
#define KIN_GRID_Y      8    // 网格Y尺寸（即使启用了也较小）
#define KIN_TASK_STACK  128  // 任务栈大小(bytes)（如果启用RTOS）
#define KIN_QUEUE_SIZE  5    // 消息队列大小

// ==================== 加速模式选择 ====================
#define KIN_ACCEL_OFF           0  // 关闭加速
#define KIN_ACCEL_FAST_MATH     1  // 仅快速数学
#define KIN_ACCEL_CCRAM         2  // 仅CCRAM优化
#define KIN_ACCEL_MATH_CCRAM    3  // 快速数学+CCRAM
#define KIN_ACCEL_FULL          4  // 完全加速

#ifndef KIN_CURRENT_ACCELERATION
#define KIN_CURRENT_ACCELERATION KIN_ACCEL_FULL
#endif

// ==================== 根据加速模式启用相应功能 ====================
#if KIN_CURRENT_ACCELERATION == KIN_ACCEL_OFF
    #undef KIN_USE_FAST
    #define KIN_USE_FAST 0
    #undef KIN_USE_CCRAM
    #define KIN_USE_CCRAM 0
#elif KIN_CURRENT_ACCELERATION == KIN_ACCEL_FAST_MATH
    #undef KIN_USE_FAST
    #define KIN_USE_FAST 1
    #undef KIN_USE_CCRAM
    #define KIN_USE_CCRAM 0
#elif KIN_CURRENT_ACCELERATION == KIN_ACCEL_CCRAM
    #undef KIN_USE_FAST
    #define KIN_USE_FAST 0
    #undef KIN_USE_CCRAM
    #define KIN_USE_CCRAM 1
#elif KIN_CURRENT_ACCELERATION == KIN_ACCEL_MATH_CCRAM
    #undef KIN_USE_FAST
    #define KIN_USE_FAST 1
    #undef KIN_USE_CCRAM
    #define KIN_USE_CCRAM 1
#elif KIN_CURRENT_ACCELERATION == KIN_ACCEL_FULL
    #undef KIN_USE_FAST
    #define KIN_USE_FAST 1
    #undef KIN_USE_CCRAM
    #define KIN_USE_CCRAM 1
#endif

// ==================== 调试配置 ====================
#define KIN_DEBUG       0   // 调试输出
#define KIN_ASSERT      0   // 断言检查（禁用以节省空间）

// ==================== 自动平台检测 ====================
#if KIN_PLATFORM_AUTO
    #if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
        defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || defined(STM32F439xx) || \
        defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
        #undef KIN_PLATFORM_STM32F4
        #define KIN_PLATFORM_STM32F4 1
    #elif defined(STM32F767xx) || defined(STM32F769xx) || defined(STM32F777xx) || defined(STM32F779xx)
        #undef KIN_PLATFORM_STM32F7
        #define KIN_PLATFORM_STM32F7 1
    #elif defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H753xx) || defined(STM32H7B0xx)
        #undef KIN_PLATFORM_STM32H7
        #define KIN_PLATFORM_STM32H7 1
    #else
        #warning "Unsupported platform, disabling CCRAM"
        #undef KIN_USE_CCRAM
        #define KIN_USE_CCRAM 0
    #endif
#endif

// 平台特定优化
#if KIN_PLATFORM_STM32F4
    #define KIN_MATH_LEVEL 2
#elif KIN_PLATFORM_STM32H7
    #define KIN_MATH_LEVEL 3
#else
    #define KIN_MATH_LEVEL 1
#endif

#if !KIN_USE_CCRAM
    #undef KIN_USE_GRID
    #define KIN_USE_GRID 0
#endif
