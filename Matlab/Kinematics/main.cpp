/* main.cpp - 使用示例 */
#include "kinematics_benchmark.h"

// FreeRTOS任务句柄
TaskHandle_t bench_task_handle = nullptr;

// 基准测试任务
void benchmark_task(void* params) {
    (void)params;

    print("===== Kinematics Benchmark =====\r\n");

    // 创建基准测试实例
    kinematics_benchmark bench;

    // 运行完整基准测试
    auto result = bench.run_full_benchmark(1000);

    print("===== Final Results =====\r\n");
    print("AVG_TIME: %.2f us\r\n", result.avg_time_us);
    print("CACHE_HIT: %.1f%%\r\n", result.cache_hit_rate);
    print("VALID: %u/%u\r\n", result.valid_results, result.test_points * result.iterations);
    print("MEM_USED: %u bytes\r\n", result.memory_used);

    vTaskDelete(nullptr);
}

void RM_RTOS_Init(void) {
    // 初始化串口打印
    print_use_uart(&huart1, true, 921600);

    print("Starting Kinematics System...\r\n");

    // 创建基准测试任务
    xTaskCreate(
        benchmark_task,
        "BenchmarkTask",
        512,
        nullptr,
        tskIDLE_PRIORITY + 2,
        &bench_task_handle
    );

    // 启动调度器
    vTaskStartScheduler();
}