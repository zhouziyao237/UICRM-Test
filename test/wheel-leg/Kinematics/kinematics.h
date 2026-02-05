/* kinematics.h - 运动学库抽象基类 */
#pragma once

#include "kinematics_config.h"
#include "bsp_print.h"
#include "bsp_math.h"
#include <cstdint>
#include <cstring>
#include <cmath>

#if KIN_USE_RTOS
#include "bsp_thread.h"
#endif

// 根据加速模式定义内存分配宏
#if KIN_USE_CCRAM && defined(MATH_USE_CCRAM)
#include "bsp_ccram.h"
// 确保内存对齐的CCRAM分配
#define KIN_MALLOC(size) CCRAM_Malloc_Aligned(size, 4)
#define KIN_CALLOC(num, size) CCRAM_Calloc_Aligned(num, size, 4)
#define KIN_FREE(ptr) CCRAM_Free(ptr)
#else
// 标准内存分配，确保4字节对齐
#define KIN_MALLOC(size) malloc_aligned(size, 4)
#define KIN_CALLOC(num, size) calloc_aligned(num, size, 4)
#define KIN_FREE(ptr) free(ptr)
#endif

// ==================== 通用数据类型 ====================
struct kin_point2d_t {
    float x;
    float y;

    kin_point2d_t(float x_ = 0, float y_ = 0) : x(x_), y(y_) {}
} __attribute__((aligned(4)));  // 确保4字节对齐

struct kin_workspace_t {
    float x_min, x_max;
    float y_min, y_max;

    kin_workspace_t(float x0 = -50, float x1 = 150, float y0 = 0, float y1 = 120)
        : x_min(x0), x_max(x1), y_min(y0), y_max(y1) {}

    bool contains(float x, float y) const {
        return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
    }
} __attribute__((aligned(4)));  // 确保4字节对齐

// ==================== 逆运动学结果 ====================
struct kin_ik_result_t {
    float alpha;        // 左侧关节角度(rad)
    float beta;         // 右侧关节角度(rad)

    struct {
        float theta1;  // 左侧夹角(0.01rad精度)
        float theta2;  // 右侧夹角(0.01rad精度)
    } angles;

    struct {
        uint8_t valid : 1;   // 结果有效
        uint8_t cached : 1;  // 来自缓存
        uint8_t error : 1;   // 计算错误
        uint8_t reserved : 5;
    } flags;

    uint16_t compute_us;  // 计算时间(μs)

    kin_ik_result_t() : alpha(0), beta(0), compute_us(0) {
        flags.valid = 0;
        flags.cached = 0;
        flags.error = 0;
        angles.theta1 = angles.theta2 = 0;
    }
} __attribute__((aligned(4)));  // 确保4字节对齐

// ==================== 正运动学结果 ====================
struct kin_fk_result_t {
    kin_point2d_t solutions[2];
    uint8_t solution_count;
    uint8_t valid;

    kin_fk_result_t() : solution_count(0), valid(0) {
        solutions[0] = kin_point2d_t(0, 0);
        solutions[1] = kin_point2d_t(0, 0);
    }
} __attribute__((aligned(4)));  // 确保4字节对齐

// ==================== 性能统计 ====================
struct kin_perf_stats_t {
    uint32_t total_calls;
    uint32_t cache_hits;
    uint32_t grid_hits;
    uint32_t fast_path_hits;
    uint32_t batch_calls;
    uint16_t avg_time_us;
    uint16_t max_time_us;
    uint16_t min_time_us;
    float cache_hit_rate;

    kin_perf_stats_t() : total_calls(0), cache_hits(0), grid_hits(0),
                        fast_path_hits(0), batch_calls(0), avg_time_us(0),
                        max_time_us(0), min_time_us(0), cache_hit_rate(0) {}
} __attribute__((aligned(4)));  // 确保4字节对齐

// ==================== 运动学基类 ====================
class kinematics {
public:
    // 构造函数/析构函数
    kinematics();
    virtual ~kinematics();

    // 禁用拷贝构造函数和赋值运算符
    kinematics(const kinematics&) = delete;
    kinematics& operator=(const kinematics&) = delete;

    // 移动构造函数和赋值运算符
    kinematics(kinematics&& other) noexcept;
    kinematics& operator=(kinematics&& other) noexcept;

    // ========== 核心接口（纯虚函数） ==========
    virtual kin_ik_result_t inverseKinematics(float x, float y) = 0;
    virtual kin_fk_result_t forwardKinematics(float alpha, float beta) = 0;
    virtual bool setModel(float L1, float L2, float L3, float L4, float L5) = 0;
    virtual bool isInitialized() const = 0;

    // ========== 通用功能接口 ==========
    virtual void setWorkspace(float x_min, float x_max, float y_min, float y_max);
    virtual void enableCache(bool enable = true);
    virtual void enableFastMath(bool enable = true);
    virtual void enableGrid(bool enable = true);
    virtual void clearCache();

    // 性能统计
    virtual kin_perf_stats_t getStats() const;
    virtual void resetStats();

    // 批量计算
    virtual uint32_t inverseKinematicsBatch(const float* x_array, const float* y_array,
                                           kin_ik_result_t* results, uint32_t count);

    // 系统信息
    virtual const char* getVersion() const;
    virtual const char* getConfigInfo() const;
    virtual bool isCCRAMEnabled() const;
    virtual void getMemoryUsage(uint32_t* used, uint32_t* free) const;

    // ========== RTOS支持 ==========
#if KIN_USE_RTOS
    // 任务消息类型
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

        void* completion_semaphore;
    } kin_task_msg_t;

    // 创建运动学任务
    static bool createTask(const char* task_name, uint32_t priority);

    // 发送计算请求（异步）
    bool requestInverseKinematics(float x, float y, kin_ik_result_t* result,
                                  uint32_t timeout_ms = 100);

    // 发送批量计算请求
    bool requestBatchInverseKinematics(const float* x_array, const float* y_array,
                                       kin_ik_result_t* results, uint32_t count,
                                       uint32_t* valid_count, uint32_t timeout_ms = 200);

    // 等待任务完成
    bool waitForCompletion(uint32_t timeout_ms = 100);
#endif

protected:
    // ========== 受保护成员变量 ==========
    kin_workspace_t workspace_;
    kin_perf_stats_t stats_;
    bool cache_enabled_;

    // 根据宏定义决定是否包含加速相关变量
    #if KIN_USE_FAST
    bool fast_math_enabled_;
    #else
    static constexpr bool fast_math_enabled_ = false;
    #endif

    #if KIN_USE_GRID
    bool grid_enabled_;
    #else
    static constexpr bool grid_enabled_ = false;
    #endif

    // 缓存系统
    struct kin_cache_entry_t {
        float x, y;
        kin_ik_result_t result;
        uint16_t hash;
        bool valid;

        kin_cache_entry_t() : x(0), y(0), hash(0), valid(false) {}
    } __attribute__((aligned(4)));  // 确保4字节对齐

    kin_cache_entry_t* cache_table_;
    uint16_t cache_index_;
    uint16_t cache_size_;

    // 快速路径状态
    mutable uint32_t fast_path_counter_;
    mutable float last_x_, last_y_;
    mutable kin_ik_result_t last_result_;

    // 静态系统状态
    static bool system_initialized_;

#if KIN_USE_RTOS
    // RTOS相关静态成员
    static void* task_handle_;
    static void* msg_queue_;
    static void* completion_sem_;
    static kinematics* task_instance_;

    // 任务函数
    static void taskFunction(void* params);
#endif

    // ========== 受保护辅助方法 ==========

    // 内存管理
    virtual bool allocateMemory();
    virtual void freeMemory();

    // 缓存管理
    uint16_t computeHash(float x, float y) const;
    bool getFromCache(float x, float y, kin_ik_result_t& result) const;
    void addToCache(float x, float y, const kin_ik_result_t& result);

    // 工作空间检查
    bool inWorkspace(float x, float y) const;

    // 快速路径检查
    bool checkFastPath(float x, float y, kin_ik_result_t& result) const;

    // 性能统计更新
    void updateStats(uint16_t compute_time, bool cached, bool fast_path);

    // 设置最后结果
    void setLastResult(float x, float y, const kin_ik_result_t& result) const;

    // 数学工具函数 - 根据加速模式条件编译
    float normalizeAngle(float angle) const;
    float clamp(float value, float min_val, float max_val) const;

    #if KIN_USE_FAST
    float fastSqrt(float x) const;
    float fastSin(float x) const;
    float fastCos(float x) const;
    float fastAtan2(float y, float x) const;
    #else
    // 如果未启用快速数学，则直接使用标准数学函数
    float fastSqrt(float x) const { return sqrtf(x); }
    float fastSin(float x) const { return sinf(x); }
    float fastCos(float x) const { return cosf(x); }
    float fastAtan2(float y, float x) const { return atan2f(y, x); }
    #endif

private:
    // 私有工具函数
    static uint32_t getMicroseconds();
    static uint32_t getMilliseconds();
};

// 额外的内存对齐函数（如果不在其他地方定义）
inline void* malloc_aligned(size_t size, size_t alignment) {
    void* ptr = malloc(size + alignment - 1 + sizeof(void*));
    if (!ptr) return nullptr;

    void** aligned_ptr = (void**)(((uintptr_t)ptr + sizeof(void*) + alignment - 1) & ~(alignment - 1));
    aligned_ptr[-1] = ptr;
    return aligned_ptr;
}

inline void* calloc_aligned(size_t num, size_t size, size_t alignment) {
    size_t total_size = num * size;
    void* ptr = malloc_aligned(total_size, alignment);
    if (ptr) memset(ptr, 0, total_size);
    return ptr;
}
