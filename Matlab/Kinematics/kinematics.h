/* kinematics.h - 高性能运动学解算库 */
#pragma once

#include <cstdio>
#include <cstring>

#include "bsp_print.h"
#include "kinematics_config.h"
#include "math.h"
#include "utils.h"

// FreeRTOS支持
#if KINEMATICS_USE_FREERTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#endif

// ==================== 平台适配宏 ====================
#if KINEMATICS_USE_CCRAM && MATH_USE_CCRAM
#define KINEMATICS_MALLOC(size) CCRAM_Malloc(size)
#define KINEMATICS_CALLOC(num, size) CCRAM_Calloc(num, size)
#define KINEMATICS_FREE(ptr) CCRAM_Free(ptr)
#define KINEMATICS_SECTION_CCRAM
#else
#define KINEMATICS_MALLOC(size) malloc(size)
#define KINEMATICS_CALLOC(num, size) calloc(num, size)
#define KINEMATICS_FREE(ptr) free(ptr)
#define KINEMATICS_SECTION_CCRAM
#endif

// ==================== 数据类型定义 ====================
// 2D点（通用）
typedef struct {
    float x;
    float y;
} kin_point2d_t;

// 模型参数（内存对齐）
typedef struct alignas(8) {
    // 基础参数
    float L1, L2, L3, L4, L5;

    // 预计算值
    float L5_half;
    float L1_sq, L2_sq, L3_sq, L4_sq, L5_sq;
    float inv_2L1, inv_2L3;

    // 工作空间
    struct {
        float x_min, x_max;
        float y_min, y_max;
    } workspace;
} kin_model_t;

// 逆解结果（紧凑）
typedef struct {
    // 关节角度
    float alpha;  // 左侧关节角度(rad)
    float beta;   // 右侧关节角度(rad)

    // 中间夹角（可选）
    struct {
        uint16_t theta1;  // 左侧夹角(0.01rad精度)
        uint16_t theta2;  // 右侧夹角(0.01rad精度)
    } angles;

    // 状态标志
    struct {
        uint8_t valid : 1;   // 结果有效
        uint8_t cached : 1;  // 来自缓存
        uint8_t error : 1;   // 计算错误
        uint8_t reserved : 5;
    } flags;

    // 性能信息
    uint16_t compute_us;  // 计算时间(μs)
} kin_ik_result_t;

// 正解结果
typedef struct {
    kin_point2d_t solutions[2];
    uint8_t solution_count;
    uint8_t valid;
} kin_fk_result_t;

// ==================== 运动学解算器类 ====================
class kinematics {
  public:
    // 构造函数/析构函数
    kinematics();
    ~kinematics();

    // 禁用拷贝（内存管理需要）
    kinematics(const kinematics&) = delete;
    kinematics& operator=(const kinematics&) = delete;

    // 移动构造函数
    kinematics(kinematics&& other) noexcept;
    kinematics& operator=(kinematics&& other) noexcept;

    /* ========== 配置方法 ========== */

    // 设置模型参数
    bool setModel(float L1, float L2, float L3, float L4, float L5);

    // 设置工作空间
    void setWorkspace(float x_min, float x_max, float y_min, float y_max);

    // 启用/禁用优化
    void enableCache(bool enable = true);
    void enableFastMath(bool enable = true);
    void enableGrid(bool enable = true);

    /* ========== 核心解算方法 ========== */

    // 逆运动学
    kin_ik_result_t inverseKinematics(float x, float y);

    // 逆运动学（带缓存）
    kin_ik_result_t inverseKinematicsCached(float x, float y);

    // 正运动学
    kin_fk_result_t forwardKinematics(float alpha, float beta);

    // 批量解算（提高效率）
    uint32_t inverseKinematicsBatch(const float* x_array, const float* y_array,
                                    kin_ik_result_t* results, uint32_t count);

/* ========== FreeRTOS任务接口 ========== */
#if KINEMATICS_USE_FREERTOS

    // 任务消息
    typedef struct {
        enum { MSG_COMPUTE_IK, MSG_COMPUTE_BATCH, MSG_UPDATE_MODEL, MSG_GET_STATS } type;

        union {
            struct {
                float x, y;
                kin_ik_result_t* result_ptr;
            } ik_data;

            struct {
                const float* x_array;
                const float* y_array;
                kin_ik_result_t* results;
                uint32_t count;
                uint32_t* valid_count;
            } batch_data;

            struct {
                float L[5];
            } model_data;
        };

        SemaphoreHandle_t completion_semaphore;
    } kin_task_msg_t;

    // 创建运动学任务
    static bool createTask(const char* task_name = "KinematicsTask",
                           UBaseType_t priority = tskIDLE_PRIORITY + 2);

    // 发送计算请求（异步）
    bool requestInverseKinematics(float x, float y, kin_ik_result_t* result,
                                  TickType_t timeout = portMAX_DELAY);

    // 发送批量计算请求
    bool requestBatchInverseKinematics(const float* x_array, const float* y_array,
                                       kin_ik_result_t* results, uint32_t count,
                                       uint32_t* valid_count, TickType_t timeout = portMAX_DELAY);

    // 等待任务完成
    bool waitForCompletion(TickType_t timeout = portMAX_DELAY);

#endif

    /* ========== 性能监控 ========== */

    // 性能统计
    typedef struct {
        uint32_t total_calls;
        uint32_t cache_hits;
        uint32_t grid_hits;
        uint32_t fast_path_hits;
        uint32_t batch_calls;
        uint16_t avg_time_us;
        uint16_t max_time_us;
        uint16_t min_time_us;
        float cache_hit_rate;
    } kin_perf_stats_t;

    // 获取统计信息
    kin_perf_stats_t getStats() const;

    // 重置统计
    void resetStats();

    // 获取内存使用情况
    void getMemoryUsage(uint32_t* used, uint32_t* free) const;

    /* ========== 系统信息 ========== */

    // 获取配置信息
    static const char* getConfigInfo();

    // 获取版本信息
    static const char* getVersion();

    // 检查系统状态
    bool isInitialized() const;
    bool isCCRAMEnabled() const;

  protected:
    // 缓存条目
    typedef struct {
        float x, y;
        kin_ik_result_t result;
        uint16_t hash;
        bool valid;
    } kin_cache_entry_t;

    // 预计算网格
    typedef struct {
        float x_min, x_max;
        float y_min, y_max;
        float x_step, y_step;
        uint16_t grid_x, grid_y;
        kin_ik_result_t* data;
        bool enabled;
    } kin_grid_t;

  private:
    /* ========== 内部数据结构 ========== */

    // 计算工作区
    struct CalcWorkspace {
        // 输入
        float x, y;

        // 中间变量（重用内存）
        struct {
            float x_sq, y_sq;
            float t, t_sq;
            float K1, K2;
            float R1, R2;
            float phi_beta, phi_alpha;
            float arg_beta, arg_alpha;
        } vars;
    };

    /* ========== 成员变量 ========== */

    // 模型参数（根据配置选择内存区域）
    kin_model_t* model_ptr_;

    // 缓存系统
    kin_cache_entry_t* cache_table_;
    uint16_t cache_index_;
    uint16_t cache_size_;
    bool cache_enabled_;

    // 预计算网格
    kin_grid_t* grid_ptr_;
    bool grid_enabled_;

    // 性能统计
    kin_perf_stats_t* stats_ptr_;

    // 工作区（栈上分配）
    mutable CalcWorkspace workspace_;

    // 快速数学开关
    bool fast_math_enabled_;

/* ========== FreeRTOS相关 ========== */
#if KINEMATICS_USE_FREERTOS

    // 任务句柄和队列
    static TaskHandle_t task_handle_;
    static QueueHandle_t msg_queue_;
    static SemaphoreHandle_t completion_sem_;
    static kinematics* task_instance_;

    // 任务函数
    static void taskFunction(void* params);

#endif

    /* ========== 内部计算方法 ========== */

    // 快速预计算
    void precomputeVars(float x, float y, CalcWorkspace& ws) const;

    // 核心计算
    kin_ik_result_t computeFromVars(const CalcWorkspace& ws) const;

    // 辅助函数
    kin_point2d_t calculatePointA(float alpha) const;
    kin_point2d_t calculatePointC(float beta) const;
    bool checkSolutionExistence(float arg_beta, float arg_alpha) const;

    // 缓存管理
    uint16_t computeHash(float x, float y) const;
    bool getFromCache(float x, float y, kin_ik_result_t& result) const;
    void addToCache(float x, float y, const kin_ik_result_t& result);

    // 网格插值
    kin_ik_result_t interpolateFromGrid(float x, float y) const;

    // 快速路径检查
    bool checkFastPath(float x, float y, kin_ik_result_t& result) const;

    // 内存管理
    bool allocateMemory();
    void freeMemory();
    void updateModelPrecompute();
    void clearCache();

    // 性能监控辅助
    void updateStats(uint16_t compute_time, bool cached, bool fast_path);

    /* ========== 静态成员 ========== */

    // 快速路径状态
    static kin_ik_result_t last_result_;
    static float last_x_, last_y_;
    static uint32_t fast_path_counter_;

    // 系统状态
    static bool system_initialized_;
};
