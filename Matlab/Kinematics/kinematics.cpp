/* kinematics.cpp - 运动学基类实现 */
#include "kinematics.h"

#include "bsp_os.h"
#include "bsp_print.h"

#if KIN_USE_RTOS
#include "bsp_thread.h"
#include "FreeRTOS.h"      // 添加 FreeRTOS 头文件
#include "task.h"          // 添加 task 相关头文件
#endif

#include <algorithm>
#include <cstring>

// ==================== 静态成员初始化 ====================
bool kinematics::system_initialized_ = false;

#if KIN_USE_RTOS
void* kinematics::task_handle_ = nullptr;
void* kinematics::msg_queue_ = nullptr;
void* kinematics::completion_sem_ = nullptr;
kinematics* kinematics::task_instance_ = nullptr;
#endif

// ==================== 工具函数 ====================

uint32_t kinematics::getMicroseconds() {
    #ifdef BSP_OS_H
    return static_cast<uint32_t>(bsp::GetHighresTickMicroSec());
    #else
    // 如果没有bsp_os，使用HAL库
    return HAL_GetTick() * 1000;
    #endif
}

uint32_t kinematics::getMilliseconds() {
    return HAL_GetTick();
}

// ==================== 构造函数/析构函数 ====================

kinematics::kinematics() :
    workspace_(-50.0f, 150.0f, 0.0f, 120.0f),
    cache_enabled_(KIN_USE_CACHE),
    #if KIN_USE_FAST
    fast_math_enabled_(KIN_USE_FAST),
    #endif
    #if KIN_USE_GRID
    grid_enabled_(KIN_USE_GRID),
    #endif
    cache_table_(nullptr),
    cache_index_(0),
    cache_size_(KIN_CACHE_SIZE),
    fast_path_counter_(0),
    last_x_(0.0f),
    last_y_(0.0f)
{
    // 初始化数学库
    if (!system_initialized_) {
        #ifdef MATH_USE_CCRAM
        if (KIN_MATH_LEVEL >= 2) {
            MathSetPerformanceMode(MATH_PERF_ACCELERATED_CCRAM);
        } else if (KIN_MATH_LEVEL == 1) {
            MathSetPerformanceMode(MATH_PERF_ACCELERATED);
        } else {
            MathSetPerformanceMode(MATH_PERF_NORMAL);
        }
        #endif
        system_initialized_ = true;
    }

    // 分配内存
    if (!allocateMemory()) {
        #if KIN_DEBUG
        print("[KIN] Memory allocation failed\r\n");
        #endif
    }

    // 重置统计
    resetStats();

    #if KIN_DEBUG
    print("[KIN] Base kinematics initialized\r\n");
    #endif
}

kinematics::~kinematics() {
    freeMemory();

    #if KIN_DEBUG
    print("[KIN] Base kinematics destroyed\r\n");
    #endif
}

kinematics::kinematics(kinematics&& other) noexcept :
    workspace_(other.workspace_),
    stats_(other.stats_),
    cache_enabled_(other.cache_enabled_),
    #if KIN_USE_FAST
    fast_math_enabled_(other.fast_math_enabled_),
    #endif
    #if KIN_USE_GRID
    grid_enabled_(other.grid_enabled_),
    #endif
    cache_table_(other.cache_table_),
    cache_index_(other.cache_index_),
    cache_size_(other.cache_size_),
    fast_path_counter_(other.fast_path_counter_),
    last_x_(other.last_x_),
    last_y_(other.last_y_),
    last_result_(other.last_result_)
{
    other.cache_table_ = nullptr;
    other.cache_size_ = 0;
}

kinematics& kinematics::operator=(kinematics&& other) noexcept {
    if (this != &other) {
        freeMemory();

        workspace_ = other.workspace_;
        stats_ = other.stats_;
        cache_enabled_ = other.cache_enabled_;
        #if KIN_USE_FAST
        fast_math_enabled_ = other.fast_math_enabled_;
        #endif
        #if KIN_USE_GRID
        grid_enabled_ = other.grid_enabled_;
        #endif
        cache_table_ = other.cache_table_;
        cache_index_ = other.cache_index_;
        cache_size_ = other.cache_size_;
        fast_path_counter_ = other.fast_path_counter_;
        last_x_ = other.last_x_;
        last_y_ = other.last_y_;
        last_result_ = other.last_result_;

        other.cache_table_ = nullptr;
        other.cache_size_ = 0;
    }
    return *this;
}

// ==================== 内存管理 ====================

bool kinematics::allocateMemory() {
    // 分配缓存表
    if (cache_enabled_) {
        cache_table_ = reinterpret_cast<kin_cache_entry_t*>(
            KIN_CALLOC(cache_size_, sizeof(kin_cache_entry_t)));

        if (!cache_table_) {
            #if KIN_DEBUG
            print("[KIN] Failed to allocate cache memory\r\n");
            #endif
            cache_enabled_ = false;
            cache_size_ = 0;
        }
    }

    return true;
}

void kinematics::freeMemory() {
    if (cache_table_) {
        KIN_FREE(cache_table_);
        cache_table_ = nullptr;
    }
    cache_size_ = 0;
}

// ==================== 工作空间管理 ====================

void kinematics::setWorkspace(float x_min, float x_max, float y_min, float y_max) {
    workspace_.x_min = x_min;
    workspace_.x_max = x_max;
    workspace_.y_min = y_min;
    workspace_.y_max = y_max;

    #if KIN_DEBUG
    print("[KIN] Workspace set: X=[%.1f, %.1f], Y=[%.1f, %.1f]\r\n",
           x_min, x_max, y_min, y_max);
    #endif
}

bool kinematics::inWorkspace(float x, float y) const {
    return workspace_.contains(x, y);
}

// ==================== 缓存管理 ====================

void kinematics::enableCache(bool enable) {
    cache_enabled_ = enable && KIN_USE_CACHE;

    if (!cache_enabled_ && cache_table_) {
        clearCache();
    }

    #if KIN_DEBUG
    print("[KIN] Cache %s\r\n", cache_enabled_ ? "enabled" : "disabled");
    #endif
}

void kinematics::clearCache() {
    if (cache_table_) {
        for (uint16_t i = 0; i < cache_size_; i++) {
            cache_table_[i].valid = false;
        }
        cache_index_ = 0;
    }

    #if KIN_DEBUG
    print("[KIN] Cache cleared\r\n");
    #endif
}

uint16_t kinematics::computeHash(float x, float y) const {
    int16_t x_int = static_cast<int16_t>(x * 10.0f);  // 0.1mm精度
    int16_t y_int = static_cast<int16_t>(y * 10.0f);
    return static_cast<uint16_t>((x_int << 8) | (y_int & 0xFF));
}

bool kinematics::getFromCache(float x, float y, kin_ik_result_t& result) const {
    if (!cache_enabled_ || !cache_table_) return false;

    uint16_t hash = computeHash(x, y);

    for (uint16_t i = 0; i < cache_size_; i++) {
        const kin_cache_entry_t& entry = cache_table_[i];

        if (entry.valid && entry.hash == hash) {
            float dx = fabsf(entry.x - x);
            float dy = fabsf(entry.y - y);

            if (dx < 1.0f && dy < 1.0f) {  // 1mm精度内认为相同
                result = entry.result;
                result.flags.cached = 1;
                result.compute_us = 0;
                return true;
            }
        }
    }

    return false;
}

void kinematics::addToCache(float x, float y, const kin_ik_result_t& result) {
    if (!cache_enabled_ || !cache_table_) return;

    kin_cache_entry_t& entry = cache_table_[cache_index_];
    entry.x = x;
    entry.y = y;
    entry.result = result;
    entry.hash = computeHash(x, y);
    entry.valid = true;

    cache_index_ = (cache_index_ + 1) % cache_size_;
}

// ==================== 快速路径检查 ====================

bool kinematics::checkFastPath(float x, float y, kin_ik_result_t& result) const {
    const float FAST_THRESHOLD_SQ = 4.0f;  // 2mm平方

    float dx = x - last_x_;
    float dy = y - last_y_;
    float distance_sq = dx * dx + dy * dy;

    // 限制快速路径使用频率
    if (++fast_path_counter_ >= 5) {
        fast_path_counter_ = 0;
        last_x_ = x;
        last_y_ = y;
        return false;
    }

    if (distance_sq < FAST_THRESHOLD_SQ && last_result_.flags.valid) {
        result = last_result_;
        result.flags.cached = 1;
        result.compute_us = 0;
        return true;
    }

    last_x_ = x;
    last_y_ = y;

    return false;
}

// ==================== 性能统计 ====================

kin_perf_stats_t kinematics::getStats() const {
    kin_perf_stats_t stats = stats_;

    // 计算实时命中率
    if (stats.total_calls > 0) {
        stats.cache_hit_rate = (stats.cache_hits * 100.0f) / stats.total_calls;
    }

    return stats;
}

void kinematics::resetStats() {
    stats_ = kin_perf_stats_t();
    stats_.min_time_us = 0xFFFF;

    fast_path_counter_ = 0;
    last_x_ = last_y_ = 0.0f;
    last_result_ = kin_ik_result_t();

    #if KIN_DEBUG
    print("[KIN] Statistics reset\r\n");
    #endif
}

void kinematics::updateStats(uint16_t compute_time, bool cached, bool fast_path) {
    stats_.total_calls++;

    if (cached) stats_.cache_hits++;
    if (fast_path) stats_.fast_path_hits++;

    // 更新计算时间统计
    if (compute_time > 0 && !cached && !fast_path) {
        if (stats_.total_calls == 1) {
            stats_.min_time_us = compute_time;
            stats_.max_time_us = compute_time;
            stats_.avg_time_us = compute_time;
        } else {
            stats_.min_time_us = std::min(stats_.min_time_us, compute_time);
            stats_.max_time_us = std::max(stats_.max_time_us, compute_time);

            // 移动平均 - 使用安全的整数运算防止溢出
            uint32_t total_time = static_cast<uint32_t>(stats_.avg_time_us) * (stats_.total_calls - 1) + compute_time;
            stats_.avg_time_us = static_cast<uint16_t>(total_time / stats_.total_calls);
        }
    }
}

void kinematics::setLastResult(float x, float y, const kin_ik_result_t& result) const {
    last_x_ = x;
    last_y_ = y;
    last_result_ = result;
}

// ==================== 数学工具函数 ====================

float kinematics::normalizeAngle(float angle) const{
    // 归一化到 [0, 2π]
    while (angle < 0) angle += 2 * M_PI;
    while (angle >= 2 * M_PI) angle -= 2 * M_PI;
    return angle;
}

float kinematics::clamp(float value, float min_val, float max_val) const{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// 根据宏定义条件编译数学函数
#if KIN_USE_FAST
float kinematics::fastSqrt(float x) const{
    #ifdef MATH_USE_CCRAM
    if (fast_math_enabled_) {
        return MathSqrtFast(x);
    }
    #endif
    return sqrtf(x);
}

float kinematics::fastSin(float x) const{
    #ifdef MATH_USE_CCRAM
    if (fast_math_enabled_) {
        return MathSinFast(x);
    }
    #endif
    return sinf(x);
}

float kinematics::fastCos(float x) const{
    #ifdef MATH_USE_CCRAM
    if (fast_math_enabled_) {
        return MathCosFast(x);
    }
    #endif
    return cosf(x);
}

float kinematics::fastAtan2(float y, float x) const{
    #ifdef MATH_USE_CCRAM
    if (fast_math_enabled_) {
        return MathAtan2(y, x);
    }
    #endif
    return atan2f(y, x);
}
#endif  // KIN_USE_FAST

// ==================== 批量计算 ====================

uint32_t kinematics::inverseKinematicsBatch(const float* x_array, const float* y_array,
                                           kin_ik_result_t* results, uint32_t count) {
    if (!x_array || !y_array || !results || count == 0) {
        return 0;
    }

    uint32_t valid_count = 0;

    #if KIN_USE_RTOS
    // 批量计算时临时提高任务优先级（如果可用）
    #ifdef configMAX_PRIORITIES
    UBaseType_t original_priority = uxTaskPriorityGet(NULL);
    if (original_priority < configMAX_PRIORITIES - 1) {
        vTaskPrioritySet(NULL, original_priority + 1);
    }
    #endif
    #endif

    for (uint32_t i = 0; i < count; i++) {
        results[i] = inverseKinematics(x_array[i], y_array[i]);
        if (results[i].flags.valid) {
            valid_count++;
        }
    }

    #if KIN_USE_RTOS
    #ifdef configMAX_PRIORITIES
    vTaskPrioritySet(NULL, original_priority);
    #endif
    #endif

    // 更新批处理统计
    stats_.batch_calls++;

    return valid_count;
}

// ==================== 系统信息 ====================

const char* kinematics::getVersion() const {
    return KIN_VERSION;
}

const char* kinematics::getConfigInfo() const {
    static char info[256];
    snprintf(info, sizeof(info),
        "Kinematics Config: CCRAM=%d, FastMath=%d, Cache=%d, Grid=%d, RTOS=%d, "
        "MathLevel=%d, CacheSize=%d, GridSize=%dx%d, AccelMode=%d",
        KIN_USE_CCRAM, KIN_USE_FAST, KIN_USE_CACHE, KIN_USE_GRID, KIN_USE_RTOS,
        KIN_MATH_LEVEL, KIN_CACHE_SIZE, KIN_GRID_X, KIN_GRID_Y, KIN_CURRENT_ACCELERATION);
    return info;
}

bool kinematics::isCCRAMEnabled() const {
    return KIN_USE_CCRAM;
}

void kinematics::getMemoryUsage(uint32_t* used, uint32_t* free) const {
    if (used) *used = 0;
    if (free) *free = 0;

    #if KIN_USE_CCRAM && defined(CCRAM_GetStats)
    size_t used_bytes, free_bytes;
    uint32_t fragments;
    CCRAM_GetStats(&used_bytes, &free_bytes, &fragments);

    if (used) *used = used_bytes;
    if (free) *free = free_bytes;
    #endif
}

// ==================== RTOS支持 ====================
#if KIN_USE_RTOS

bool kinematics::createTask(const char* task_name, uint32_t priority) {
    if (task_handle_ != nullptr) {
        return true;  // 任务已存在
    }

    #if KIN_DEBUG
    print("[KIN] Creating task: %s, priority: %u\r\n", task_name, priority);
    #endif

    // 使用bsp_thread库创建任务
    bsp::thread_init_t thread_init = {
        .func = nullptr,  // 将在taskFunction中设置
        .args = nullptr,
        .attr = {
            .name = task_name,
            .attr_bits = 0,
            .cb_mem = nullptr,
            .cb_size = 0,
            .stack_mem = nullptr,
            .stack_size = KIN_TASK_STACK,
            .priority = osPriority_t(priority),
            .tz_module = 0,
            .reserved = 0
        }
    };

    // 创建任务
    task_handle_ = new bsp::Thread(thread_init);
    if (!task_handle_) {
        #if KIN_DEBUG
        print("[KIN] Failed to create task\r\n");
        #endif
        return false;
    }

    #if KIN_DEBUG
    print("[KIN] Task created successfully\r\n");
    #endif

    return true;
}

void kinematics::taskFunction(void* params) {
    UNUSED(params);
    // 任务函数实现
    while (1) {
        // 等待消息处理
        #if KIN_DEBUG
        print("[KIN] Task running...\r\n");
        #endif
        osDelay(pdMS_TO_TICKS(1000));
    }
}

bool kinematics::requestInverseKinematics(float x, float y, kin_ik_result_t* result,
                                          uint32_t timeout_ms) {
    #if KIN_DEBUG
    print("[KIN] RTOS request not fully implemented\r\n");
    #endif
    (void)x; (void)y; (void)result; (void)timeout_ms;
    return false;
}

bool kinematics::requestBatchInverseKinematics(const float* x_array, const float* y_array,
                                               kin_ik_result_t* results, uint32_t count,
                                               uint32_t* valid_count, uint32_t timeout_ms) {
    #if KIN_DEBUG
    print("[KIN] RTOS batch request not fully implemented\r\n");
    #endif
    (void)x_array; (void)y_array; (void)results; (void)count;
    (void)valid_count; (void)timeout_ms;
    return false;
}

bool kinematics::waitForCompletion(uint32_t timeout_ms) {
    #if KIN_DEBUG
    print("[KIN] Wait for completion not implemented\r\n");
    #endif
    (void)timeout_ms;
    return false;
}

#endif  // KIN_USE_RTOS

// ==================== 快速数学控制 ====================

void kinematics::enableFastMath(bool enable) {
    #if KIN_USE_FAST
    fast_math_enabled_ = enable && KIN_USE_FAST;

    #if KIN_DEBUG
    print("[KIN] Fast math %s\r\n", fast_math_enabled_ ? "enabled" : "disabled");
    #endif
    #else
    (void)enable;  // 防止未使用参数警告
    #endif
}

void kinematics::enableGrid(bool enable) {
    #if KIN_USE_GRID
    grid_enabled_ = enable && KIN_USE_GRID;

    #if KIN_DEBUG
    print("[KIN] Grid interpolation %s\r\n", grid_enabled_ ? "enabled" : "disabled");
    #endif
    #else
    (void)enable;  // 防止未使用参数警告
    #endif
}
