/* kinematics_benchmark.h */
#pragma once

#include "kinematics.h"

class kinematics_benchmark : public kinematics {
  public:
    // 测试结果结构
    struct benchmark_result_t {
        uint32_t test_points;    // 测试点数
        uint32_t iterations;     // 迭代次数
        uint32_t total_time_ms;  // 总时间(ms)
        float avg_time_us;       // 平均时间(μs)
        float min_time_us;       // 最短时间(μs)
        float max_time_us;       // 最长时间(μs)
        float cache_hit_rate;    // 缓存命中率(%)
        uint32_t valid_results;  // 有效结果数
        uint32_t memory_used;    // 内存使用(bytes)
        bool ccram_enabled;      // CCRAM是否启用
        bool fast_math_enabled;  // 快速数学是否启用
    };

    // 构造函数
    kinematics_benchmark();

    // 运行完整基准测试
    benchmark_result_t run_full_benchmark(uint32_t iterations = 1000);

    // 运行单项测试
    benchmark_result_t run_single_point_test(uint32_t iterations = 10000);
    benchmark_result_t run_batch_test(uint32_t batch_size = 50, uint32_t iterations = 100);
    benchmark_result_t run_workspace_coverage_test(uint32_t grid_points = 100);

    // 精度测试
    float run_accuracy_test(uint32_t test_points = 100);

    // 内存使用测试
    void run_memory_test();

// FreeRTOS任务测试
#if KINEMATICS_USE_FREERTOS
    benchmark_result_t run_freertos_task_test(uint32_t iterations = 1000);
#endif

    // 打印测试结果（英文简写）
    static void print_result(const benchmark_result_t& result, const char* test_name);
    static void print_config_info();

  private:
    // 内部运动学实例
    kinematics kin_;

    // 测试数据生成
    void generate_test_points(float* x_array, float* y_array, uint32_t count, float x_min,
                              float x_max, float y_min, float y_max);

    // 性能监控
    struct {
        uint32_t start_time_ms;
        uint32_t start_time_us;
        uint32_t total_calls;
        uint32_t valid_calls;
        uint32_t min_time_us;
        uint32_t max_time_us;
        uint32_t total_time_us;
    } perf_monitor_;

    void perf_start();
    void perf_stop(bool valid = true);
    void perf_reset();
};