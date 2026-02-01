/* main.cpp - 运动学库使用示例 */
#include "main.h"

#include "FreeRTOS.h"
#include "Model/fivebar.h"
#include "bsp_os.h"
#include "bsp_print.h"

// 示例1: 五连杆基本使用
void demo_fivebar() {
    print("=== 五连杆运动学示例 ===\r\n");
    print("当前加速模式: %d\r\n", KIN_CURRENT_ACCELERATION);

    // 创建局部实例以减少内存占用
    FiveBarLinkage kin;

    // 显示配置信息
    print("版本: %s\r\n", kin.getVersion());
    print("配置: %s\r\n\r\n", kin.getConfigInfo());

    // 设置模型参数
    if (!kin.setModel(45.0f, 80.0f, 45.0f, 80.0f, 55.0f)) {
        print("设置模型参数失败!\r\n");
        return;
    }

    // 设置工作空间
    kin.setWorkspace(-50.0f, 150.0f, 0.0f, 120.0f);

    // 启用优化功能（根据宏定义条件启用）
    kin.enableCache(KIN_USE_CACHE);
    kin.enableFastMath(KIN_USE_FAST);
    kin.enableGrid(KIN_USE_GRID);

    // 重置统计
    kin.resetStats();

    // 测试点 - 减少数量以降低内存使用
    float test_points[][2] = {
        {100.0f, 50.0f}, {80.0f, 60.0f}, {120.0f, 40.0f}};

    // 单点计算
    print("单点计算测试:\r\n");
    for (int i = 0; i < 3; i++) {
        float x = test_points[i][0];
        float y = test_points[i][1];

        kin_ik_result_t result = kin.inverseKinematics(x, y);

        print("点%d (%.1f, %.1f): ", i + 1, x, y);
        if (result.flags.valid) {
            print("α=%.2f° β=%.2f°", result.alpha * 180.0f / M_PI, result.beta * 180.0f / M_PI);

            if (result.flags.cached) {
                print(" [缓存命中]");
            }
            print("\r\n");
        } else {
            print("无效结果\r\n");
        }
    }
    print("\r\n");

    // 批量计算
    print("批量计算测试:\r\n");
    float batch_x[3], batch_y[3];
    kin_ik_result_t batch_results[3];

    for (int i = 0; i < 3; i++) {
        batch_x[i] = 100.0f + i * 5.0f;
        batch_y[i] = 50.0f + i * 4.0f;
    }

    uint32_t valid = kin.inverseKinematicsBatch(batch_x, batch_y, batch_results, 3);
    print("批量计算: %u/3 成功\r\n", valid);

    // 正运动学验证
    print("\r\n正运动学验证:\r\n");
    kin_ik_result_t ik_result = kin.inverseKinematics(100.0f, 50.0f);
    if (ik_result.flags.valid) {
        kin_fk_result_t fk_result = kin.forwardKinematics(ik_result.alpha, ik_result.beta);
        if (fk_result.valid) {
            print("逆解点: (100.0, 50.0)\r\n");
            print("正解点: (%.2f, %.2f)\r\n", fk_result.solutions[0].x, fk_result.solutions[0].y);

            float error = sqrtf(powf(fk_result.solutions[0].x - 100.0f, 2) +
                                powf(fk_result.solutions[0].y - 50.0f, 2));
            print("误差: %.4f mm\r\n", error);
        }
    }
    print("\r\n");

    // 性能统计
    kin_perf_stats_t stats = kin.getStats();
    print("性能统计:\r\n");
    print("总调用次数: %u\r\n", stats.total_calls);
    print("缓存命中: %u (%.1f%%)\r\n", stats.cache_hits, stats.cache_hit_rate);
    print("快速路径命中: %u\r\n", stats.fast_path_hits);
    print("批处理调用: %u\r\n", stats.batch_calls);
    print("平均计算时间: %u μs\r\n", stats.avg_time_us);
    print("最快: %u μs, 最慢: %u μs\r\n", stats.min_time_us, stats.max_time_us);

    // 内存使用
    uint32_t used_mem, free_mem;
    kin.getMemoryUsage(&used_mem, &free_mem);
    print("\r\n内存使用:\r\n");
    print("已使用: %u bytes\r\n", used_mem);
    print("可用: %u bytes\r\n", free_mem);
}

// 示例3: 性能对比
void demo_performance() {
    print("\r\n=== 性能对比测试 ===\r\n");

    // 创建临时实例
    FiveBarLinkage temp_kin;

    temp_kin.setModel(45, 80, 45, 80, 55);

    // 禁用缓存以测试原始性能
    temp_kin.enableCache(false);

    // 重置统计
    temp_kin.resetStats();

    // 减少测试循环次数
    const int test_count = 100;  // 从1000减少到100
    print("正在进行 %d 次计算测试...\r\n", test_count);

    uint32_t start_time = bsp::GetHighresTickMilliSec();

    for (int i = 0; i < test_count; i++) {
        float x = 80.0f + (i % 20);  // 减少范围
        float y = 40.0f + (i % 15);

        temp_kin.inverseKinematics(x, y);
    }

    uint32_t total_time = bsp::GetHighresTickMilliSec() - start_time;

    // 获取统计
    kin_perf_stats_t temp_stats = temp_kin.getStats();

    print("\r\n测试结果:\r\n");
    print("总时间: %u ms\r\n", total_time);
    print("平均每点: %.2f ms\r\n", total_time / (float)test_count);
    print("\r\n五连杆统计:\r\n");
    print("  调用次数: %u\r\n", temp_stats.total_calls);
    print("  平均时间: %u μs\r\n", temp_stats.avg_time_us);
    print("\r\nSCARA统计:\r\n");
}

void demo_system_info() {
    print("\r\n=== 系统信息 ===\r\n");

    // 临时创建实例获取信息
    FiveBarLinkage temp_kin;
    print("运动学库版本: %s\r\n", temp_kin.getVersion());
    print("编译配置:\r\n");
    print("  CCRAM: %s\r\n", temp_kin.isCCRAMEnabled() ? "启用" : "禁用");
    print("  快速数学: %s\r\n", KIN_USE_FAST ? "启用" : "禁用");
    print("  缓存: %s (%d 条目)\r\n", KIN_USE_CACHE ? "启用" : "禁用", KIN_CACHE_SIZE);
    print("  RTOS: %s\r\n", KIN_USE_RTOS ? "启用" : "禁用");
    print("  网格: %s\r\n", KIN_USE_GRID ? "启用" : "禁用");
    print("  统计: %s\r\n", KIN_USE_STATS ? "启用" : "禁用");
    print("  数学优化级别: %d\r\n", KIN_MATH_LEVEL);
    print("  加速模式: %d\r\n", KIN_CURRENT_ACCELERATION);
}

// 禁用RTOS测试（因为已禁用KIN_USE_RTOS）
#if KIN_USE_RTOS
void demo_rtos_task() {
    print("\r\n=== RTOS任务测试 ===\r\n");
    // 创建临时实例
    FiveBarLinkage temp_kin;
    temp_kin.setModel(45, 80, 45, 80, 55);

    // 创建RTOS任务
    if (temp_kin.createTask("KinematicsTask", 2)) {
        print("RTOS任务创建成功\r\n");

        // 异步计算请求
        kin_ik_result_t result;
        if (temp_kin.requestInverseKinematics(100, 50, &result, 100)) {
            if (result.flags.valid) {
                print("异步计算成功: α=%.1f°, β=%.1f°\r\n", result.alpha * 180 / M_PI,
                      result.beta * 180 / M_PI);
            }
        } else {
            print("异步计算超时\r\n");
        }
    } else {
        print("RTOS任务创建失败\r\n");
    }
}
#else
void demo_rtos_task() {
    print("\r\n=== RTOS任务测试 ===\r\n");
    print("RTOS功能已禁用以节省内存\r\n");
}
#endif

void RM_RTOS_Init(void) {
    // 初始化串口打印
    print_use_uart(&huart1, true, 912600);

    HAL_Delay(1000);
    clear_screen();
    set_cursor(0,0);
    print("test demo run\r\n");
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    print("========================================\r\n");

    // 运行示例
    demo_fivebar();
    demo_performance();
    demo_system_info();
    demo_rtos_task();

    print("========================================\r\n");

    while (1) {
        // 主循环
#if KIN_USE_RTOS
        osDelay(pdMS_TO_TICKS(1000));
#else
        HAL_Delay(1000);
#endif
    }
}
