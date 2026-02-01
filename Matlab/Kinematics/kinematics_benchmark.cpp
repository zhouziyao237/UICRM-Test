/* kinematics_benchmark.cpp */
#include "kinematics_benchmark.h"
#include "kinematics.h"  // 添加对kinematics.h的包含

#include <cmath>

#include "bsp_os.h"
#include "bsp_print.h"

// 定义缺失的宏
#ifndef BENCHMARK_GET_MS
#define BENCHMARK_GET_MS() bsp::GetHighresTickMilliSec()
#endif

#ifndef BENCHMARK_GET_US
#define BENCHMARK_GET_US() bsp::GetHighresTickMicroSec()
#endif

kinematics_benchmark::kinematics_benchmark() {
    // 初始化运动学实例
    kin_.setModel(45.0f, 80.0f, 45.0f, 80.0f, 55.0f);
    kin_.setWorkspace(-50.0f, 150.0f, 0.0f, 120.0f);

    print("[BENCH] Kinematics Benchmark Initialized\r\n");
    print_config_info();
}

kinematics_benchmark::benchmark_result_t
kinematics_benchmark::run_full_benchmark(uint32_t iterations) {
    benchmark_result_t result = {};

    print("[BENCH] ====== Full Benchmark Start ======\r\n");

    // 1. 单点性能测试
    print("[BENCH] --- Single Point Test ---\r\n");
    auto single_result = run_single_point_test(iterations);
    print_result(single_result, "SINGLE_POINT");

    // 2. 批量性能测试
    print("[BENCH] --- Batch Test ---\r\n");
    auto batch_result = run_batch_test(50, iterations / 10);
    print_result(batch_result, "BATCH_50");

    // 3. 工作空间覆盖测试
    print("[BENCH] --- Workspace Coverage Test ---\r\n");
    auto coverage_result = run_workspace_coverage_test(100);
    print_result(coverage_result, "COVERAGE");

    // 4. 精度测试
    print("[BENCH] --- Accuracy Test ---\r\n");
    float accuracy = run_accuracy_test(50);
    print("  ACC: %.4f mm avg error\r\n", accuracy);

    // 5. 内存测试
    print("[BENCH] --- Memory Test ---\r\n");
    run_memory_test();

    // 6. FreeRTOS任务测试（如果启用）
    #if KINEMATICS_USE_FREERTOS
    print("[BENCH] --- FreeRTOS Task Test ---\r\n");
    auto task_result = run_freertos_task_test(iterations / 2);
    print_result(task_result, "FREERTOS_TASK");
    #endif

    // 汇总结果
    result = single_result;  // 使用单点测试作为主要结果
    result.cache_hit_rate = kin_.getStats().cache_hit_rate;
    result.ccram_enabled = kin_.isCCRAMEnabled();
    result.fast_math_enabled = KINEMATICS_USE_FAST_MATH;

    print("[BENCH] ====== Full Benchmark Complete ======\r\n");

    return result;
}

kinematics_benchmark::benchmark_result_t
kinematics_benchmark::run_single_point_test(uint32_t iterations) {
    benchmark_result_t result = {};
    perf_reset();

    // 测试点
    const float test_points[][2] = {
        {100.0f, 50.0f},
        {80.0f, 60.0f},
        {120.0f, 40.0f},
        {90.0f, 70.0f},
        {110.0f, 30.0f}
    };
    const uint32_t num_points = 5;

    // 预热缓存
    for (uint32_t i = 0; i < 10; i++) {
        for (uint32_t j = 0; j < num_points; j++) {
            kin_.inverseKinematicsCached(test_points[j][0], test_points[j][1]);
        }
    }
    kin_.resetStats();

    uint32_t start_time_ms = BENCHMARK_GET_MS();  // 使用定义的宏

    for (uint32_t i = 0; i < iterations; i++) {
        for (uint32_t j = 0; j < num_points; j++) {
            perf_start();
            auto ik_result = kin_.inverseKinematicsCached(test_points[j][0], test_points[j][1]);
            perf_stop(ik_result.flags.valid);
        }
    }

    uint32_t total_time_ms = BENCHMARK_GET_MS() - start_time_ms;  // 使用定义的宏

    // 收集结果
    auto stats = kin_.getStats();

    result.test_points = num_points;
    result.iterations = iterations;
    result.total_time_ms = total_time_ms;
    result.avg_time_us = stats.avg_time_us;
    result.min_time_us = stats.min_time_us;
    result.max_time_us = stats.max_time_us;
    result.cache_hit_rate = stats.cache_hit_rate;
    result.valid_results = perf_monitor_.valid_calls;
    result.ccram_enabled = kin_.isCCRAMEnabled();
    result.fast_math_enabled = KINEMATICS_USE_FAST_MATH;

    return result;
}

kinematics_benchmark::benchmark_result_t
kinematics_benchmark::run_batch_test(uint32_t batch_size, uint32_t iterations) {
    benchmark_result_t result = {};

    // 分配内存
    float* x_array = new float[batch_size];
    float* y_array = new float[batch_size];
    kin_ik_result_t* results = new kin_ik_result_t[batch_size];

    // 生成测试数据（圆形轨迹）
    for (uint32_t i = 0; i < batch_size; i++) {
        float angle = (i * 2.0f * MATH_PI) / batch_size;
        x_array[i] = 100.0f + 30.0f * cosf(angle);
        y_array[i] = 60.0f + 30.0f * sinf(angle);
    }

    uint32_t start_time_ms = BENCHMARK_GET_MS();  // 使用定义的宏
    uint32_t total_valid = 0;

    for (uint32_t i = 0; i < iterations; i++) {
        uint32_t valid = kin_.inverseKinematicsBatch(x_array, y_array, results, batch_size);
        total_valid += valid;
    }

    uint32_t total_time_ms = BENCHMARK_GET_MS() - start_time_ms;  // 使用定义的宏
    auto stats = kin_.getStats();

    // 计算平均时间
    float avg_time_per_point = (total_time_ms * 1000.0f) / (iterations * batch_size);

    result.test_points = batch_size;
    result.iterations = iterations;
    result.total_time_ms = total_time_ms;
    result.avg_time_us = avg_time_per_point;
    result.valid_results = total_valid;
    result.cache_hit_rate = stats.cache_hit_rate;

    // 清理内存
    delete[] x_array;
    delete[] y_array;
    delete[] results;

    return result;
}

kinematics_benchmark::benchmark_result_t
kinematics_benchmark::run_workspace_coverage_test(uint32_t grid_points) {
    benchmark_result_t result = {};

    uint32_t points_per_dim = static_cast<uint32_t>(sqrtf(grid_points));
    uint32_t total_points = points_per_dim * points_per_dim;

    float* x_array = new float[total_points];
    float* y_array = new float[total_points];

    // 生成网格点
    float x_step = 200.0f / (points_per_dim - 1);
    float y_step = 120.0f / (points_per_dim - 1);

    uint32_t index = 0;
    for (uint32_t i = 0; i < points_per_dim; i++) {
        for (uint32_t j = 0; j < points_per_dim; j++) {
            x_array[index] = -50.0f + j * x_step;
            y_array[index] = i * y_step;
            index++;
        }
    }

    kin_ik_result_t* results = new kin_ik_result_t[total_points];

    uint32_t start_time_ms = BENCHMARK_GET_MS();  // 使用定义的宏
    uint32_t valid_count = kin_.inverseKinematicsBatch(x_array, y_array, results, total_points);
    uint32_t total_time_ms = BENCHMARK_GET_MS() - start_time_ms;  // 使用定义的宏

    // 统计成功率和时间
    float success_rate = (valid_count * 100.0f) / total_points;
    // 移除未使用的变量
    // float avg_time_us = (total_time_ms * 1000.0f) / total_points;

    // 找到最大/最小时间
    uint32_t min_time = 0xFFFFFFFF;
    uint32_t max_time = 0;
    uint32_t total_time = 0;
    uint32_t count_with_time = 0;

    for (uint32_t i = 0; i < total_points; i++) {
        if (results[i].flags.valid && results[i].compute_us > 0) {
            if (results[i].compute_us < min_time) min_time = results[i].compute_us;
            if (results[i].compute_us > max_time) max_time = results[i].compute_us;
            total_time += results[i].compute_us;
            count_with_time++;
        }
    }

    result.test_points = total_points;
    result.iterations = 1;
    result.total_time_ms = total_time_ms;
    result.avg_time_us = count_with_time > 0 ? (total_time / count_with_time) : 0;
    result.min_time_us = min_time;
    result.max_time_us = max_time;
    result.valid_results = valid_count;
    result.cache_hit_rate = success_rate;  // 这里复用为成功率

    delete[] x_array;
    delete[] y_array;
    delete[] results;

    return result;
}

float kinematics_benchmark::run_accuracy_test(uint32_t test_points) {
    float total_error = 0.0f;
    uint32_t valid_tests = 0;

    // 使用bsp::GetHighresTickMicroSec()作为随机种子
    srand(BENCHMARK_GET_US());  // 使用定义的宏

    for (uint32_t i = 0; i < test_points; i++) {
        // 在可行工作空间内随机生成点
        float x = -30.0f + (rand() % 160);  // -30 ~ 130
        float y = 10.0f + (rand() % 110);   // 10 ~ 120

        // 计算逆运动学
        auto ik_result = kin_.inverseKinematics(x, y);

        if (ik_result.flags.valid) {
            // 计算正运动学验证
            auto fk_result = kin_.forwardKinematics(ik_result.alpha, ik_result.beta);

            if (fk_result.valid) {
                // 计算误差（使用最近的点）
                float min_error = 1e6;
                for (uint8_t j = 0; j < fk_result.solution_count; j++) {
                    float dx = fk_result.solutions[j].x - x;
                    float dy = fk_result.solutions[j].y - y;
                    float error = sqrtf(dx * dx + dy * dy);

                    if (error < min_error) min_error = error;
                }

                total_error += min_error;
                valid_tests++;
            }
        }
    }

    return valid_tests > 0 ? (total_error / valid_tests) : 0.0f;
}

void kinematics_benchmark::run_memory_test() {
    print("  MEM: ");

    #if KINEMATICS_USE_CCRAM && MATH_USE_CCRAM
        size_t used, free;
        uint32_t fragments;
        CCRAM_GetStats(&used, &free, &fragments);

        print("CCRAM Used: %lu/%lu bytes (%.1f%%), Frag: %lu\r\n",
               used, used + free, (used * 100.0f) / (used + free), fragments);
    #else
        print("No CCRAM, using heap memory\r\n");
    #endif

    // 显示配置的内存使用
    uint32_t estimated_memory =
        sizeof(kin_model_t) +
        (KINEMATICS_CACHE_SIZE * sizeof(kin_cache_entry_t)) +  // 使用完全限定名
        sizeof(kin_perf_stats_t);  // 使用完全限定名

    if (KINEMATICS_GRID_ENABLED) {
        estimated_memory += sizeof(kin_grid_t) +  // 使用完全限定名
                           (KINEMATICS_GRID_SIZE_X * KINEMATICS_GRID_SIZE_Y * sizeof(kin_ik_result_t));
    }

    print("  MEM: Estimated usage: %u bytes\r\n", estimated_memory);
}

#if KINEMATICS_USE_FREERTOS
kinematics_benchmark::benchmark_result_t
kinematics_benchmark::run_freertos_task_test(uint32_t iterations) {
    benchmark_result_t result = {};

    // 确保任务已创建
    if (!kinematics::createTask("BenchmarkTask", tskIDLE_PRIORITY + 3)) {
        print("[BENCH] Failed to create FreeRTOS task\r\n");
        return result;
    }

    // 准备测试数据
    const float test_x = 100.0f;
    const float test_y = 50.0f;
    kin_ik_result_t async_result;

    uint32_t start_time_ms = BENCHMARK_GET_MS();  // 使用定义的宏
    uint32_t valid_count = 0;

    for (uint32_t i = 0; i < iterations; i++) {
        if (kin_.requestInverseKinematics(test_x, test_y, &async_result, 100)) {
            if (async_result.flags.valid) {
                valid_count++;
            }
        }
    }

    uint32_t total_time_ms = BENCHMARK_GET_MS() - start_time_ms;  // 使用定义的宏

    result.test_points = 1;
    result.iterations = iterations;
    result.total_time_ms = total_time_ms;
    result.avg_time_us = (total_time_ms * 1000.0f) / iterations;
    result.valid_results = valid_count;
    result.cache_hit_rate = (valid_count * 100.0f) / iterations;

    return result;
}
#endif

void kinematics_benchmark::print_result(const benchmark_result_t& result, const char* test_name) {
    print("  TST: %s\r\n", test_name);
    print("  PTS: %u, ITER: %u\r\n", result.test_points, result.iterations);
    print("  TIM: %u ms total, %.2f us avg\r\n", result.total_time_ms, result.avg_time_us);

    if (result.min_time_us > 0 && result.max_time_us > 0) {
        print("  MIN: %.2f us, MAX: %.2f us\r\n", result.min_time_us, result.max_time_us);
    }

    print("  HIT: %.1f%%, VALID: %u/%u\r\n",
           result.cache_hit_rate, result.valid_results,
           result.test_points * result.iterations);

    if (result.ccram_enabled) {
        print("  OPT: CCRAM=Y");
    } else {
        print("  OPT: CCRAM=N");
    }

    if (result.fast_math_enabled) {
        print(", FastMath=Y\r\n");
    } else {
        print(", FastMath=N\r\n");
    }
}

void kinematics_benchmark::print_config_info() {
    print("[BENCH] Config Info:\r\n");
    print("  PLAT: STM32F%d\r\n",
           KINEMATICS_PLATFORM_STM32F4 ? 4 :
           KINEMATICS_PLATFORM_STM32F7 ? 7 :
           KINEMATICS_PLATFORM_STM32H7 ? 7 : 0);
    print("  OPT: CCRAM=%d, FastMath=%d, Cache=%d\r\n",
           KINEMATICS_USE_CCRAM, KINEMATICS_USE_FAST_MATH, KINEMATICS_CACHE_ENABLED);
    print("  SIZ: Cache=%d, Grid=%dx%d\r\n",
           KINEMATICS_CACHE_SIZE, KINEMATICS_GRID_SIZE_X, KINEMATICS_GRID_SIZE_Y);
    print("  RTOS: FreeRTOS=%d, Queue=%d\r\n",
           KINEMATICS_USE_FREERTOS, KINEMATICS_QUEUE_SIZE);
}

void kinematics_benchmark::perf_start() {
    perf_monitor_.start_time_us = bsp::GetHighresTickMilliSec();
}

void kinematics_benchmark::perf_stop(bool valid) {
    uint32_t end_time_us = bsp::GetHighresTickMilliSec();
    uint32_t elapsed_us = end_time_us - perf_monitor_.start_time_us;

    perf_monitor_.total_calls++;

    if (valid) {
        perf_monitor_.valid_calls++;

        if (elapsed_us < perf_monitor_.min_time_us) {
            perf_monitor_.min_time_us = elapsed_us;
        }

        if (elapsed_us > perf_monitor_.max_time_us) {
            perf_monitor_.max_time_us = elapsed_us;
        }

        perf_monitor_.total_time_us += elapsed_us;
    }
}

void kinematics_benchmark::perf_reset() {
    memset(&perf_monitor_, 0, sizeof(perf_monitor_));
    perf_monitor_.min_time_us = 0xFFFFFFFF;
}

void kinematics_benchmark::generate_test_points(float* x_array, float* y_array,
                                               uint32_t count, float x_min, float x_max,
                                               float y_min, float y_max) {
    float x_range = x_max - x_min;
    float y_range = y_max - y_min;

    srand(bsp::GetHighresTickMicroSec());

    for (uint32_t i = 0; i < count; i++) {
        x_array[i] = x_min + (rand() / (float)RAND_MAX) * x_range;
        y_array[i] = y_min + (rand() / (float)RAND_MAX) * y_range;
    }
}
