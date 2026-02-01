/* kinematics.cpp - 运动学库实现 */
#include "kinematics.h"
#include <cstring>
#include <algorithm>

// ==================== 静态成员初始化 ====================
kin_ik_result_t kinematics::last_result_ = {};
float kinematics::last_x_ = 0.0f;
float kinematics::last_y_ = 0.0f;
uint32_t kinematics::fast_path_counter_ = 0;
bool kinematics::system_initialized_ = false;

#if KINEMATICS_USE_FREERTOS
TaskHandle_t kinematics::task_handle_ = nullptr;
QueueHandle_t kinematics::msg_queue_ = nullptr;
SemaphoreHandle_t kinematics::completion_sem_ = nullptr;
kinematics* kinematics::task_instance_ = nullptr;
#endif

// ==================== 构造函数/析构函数 ====================

kinematics::kinematics()
    : model_ptr_(nullptr)
    , cache_table_(nullptr)
    , cache_index_(0)
    , cache_size_(KINEMATICS_CACHE_SIZE)
    , cache_enabled_(KINEMATICS_CACHE_ENABLED)
    , grid_ptr_(nullptr)
    , grid_enabled_(KINEMATICS_GRID_ENABLED)
    , stats_ptr_(nullptr)
    , fast_math_enabled_(KINEMATICS_USE_FAST_MATH) {
    
    // 初始化数学库
    if (!system_initialized_) {
        MathPerformanceMode mode;
        
        // 根据配置选择数学库模式
        switch (KINEMATICS_MATH_LEVEL) {
            case 0: mode = MATH_PERF_NORMAL; break;
            case 1: mode = MATH_PERF_ACCELERATED; break;
            case 2: mode = MATH_PERF_ACCELERATED_CCRAM; break;
            case 3: mode = MATH_PERF_ACCELERATED_CCRAM; break;  // 最高优化
            default: mode = MATH_PERF_AUTO; break;
        }
        
        MathInit(mode);
        system_initialized_ = true;
    }
    
    // 分配内存
    if (!allocateMemory()) {
        // 内存分配失败
        #if KINEMATICS_ENABLE_DEBUG
            print("[KIN] Memory allocation failed\r\n");
        #endif
        return;
    }
    
    // 设置默认参数
    setModel(45.0f, 80.0f, 45.0f, 80.0f, 55.0f);
    setWorkspace(-50.0f, 150.0f, 0.0f, 120.0f);
    
    // 初始化统计
    resetStats();
    
    #if KINEMATICS_ENABLE_DEBUG
        print("[KIN] Solver initialized, CCRAM: %s\r\n", 
               KINEMATICS_USE_CCRAM ? "enabled" : "disabled");
    #endif
}

kinematics::~kinematics() {
    freeMemory();
    
    #if KINEMATICS_ENABLE_DEBUG
        print("[KIN] Solver destroyed\r\n");
    #endif
}

kinematics::kinematics(kinematics&& other) noexcept
    : model_ptr_(other.model_ptr_)
    , cache_table_(other.cache_table_)
    , cache_index_(other.cache_index_)
    , cache_size_(other.cache_size_)
    , cache_enabled_(other.cache_enabled_)
    , grid_ptr_(other.grid_ptr_)
    , grid_enabled_(other.grid_enabled_)
    , stats_ptr_(other.stats_ptr_)
    , workspace_(other.workspace_)
    , fast_math_enabled_(other.fast_math_enabled_) {
    
    // 清空原对象
    other.model_ptr_ = nullptr;
    other.cache_table_ = nullptr;
    other.grid_ptr_ = nullptr;
    other.stats_ptr_ = nullptr;
    other.cache_size_ = 0;
}

kinematics& kinematics::operator=(kinematics&& other) noexcept {
    if (this != &other) {
        // 释放当前内存
        freeMemory();
        
        // 转移所有权
        model_ptr_ = other.model_ptr_;
        cache_table_ = other.cache_table_;
        cache_index_ = other.cache_index_;
        cache_size_ = other.cache_size_;
        cache_enabled_ = other.cache_enabled_;
        grid_ptr_ = other.grid_ptr_;
        grid_enabled_ = other.grid_enabled_;
        stats_ptr_ = other.stats_ptr_;
        workspace_ = other.workspace_;
        fast_math_enabled_ = other.fast_math_enabled_;
        
        // 清空原对象
        other.model_ptr_ = nullptr;
        other.cache_table_ = nullptr;
        other.grid_ptr_ = nullptr;
        other.stats_ptr_ = nullptr;
        other.cache_size_ = 0;
    }
    return *this;
}

// ==================== 内存管理 ====================

bool kinematics::allocateMemory() {
    // 分配模型参数
    model_ptr_ = reinterpret_cast<kin_model_t*>(
        KINEMATICS_CALLOC(1, sizeof(kin_model_t)));

    if (!model_ptr_) {
        #if KINEMATICS_ENABLE_DEBUG
            print("[KIN] Failed to allocate model memory\r\n");
        #endif
        return false;
    }

    // 分配缓存表
    cache_table_ = reinterpret_cast<kin_cache_entry_t*>(
        KINEMATICS_CALLOC(cache_size_, sizeof(kin_cache_entry_t)));

    if (!cache_table_) {
        KINEMATICS_FREE(model_ptr_);
        model_ptr_ = nullptr;

        #if KINEMATICS_ENABLE_DEBUG
            print("[KIN] Failed to allocate cache memory\r\n");
        #endif
        return false;
    }

    // 初始化缓存
    memset(cache_table_, 0, cache_size_ * sizeof(kin_cache_entry_t));

    // 分配性能统计
    stats_ptr_ = reinterpret_cast<kin_perf_stats_t*>(
        KINEMATICS_CALLOC(1, sizeof(kin_perf_stats_t)));

    if (!stats_ptr_) {
        KINEMATICS_FREE(model_ptr_);
        KINEMATICS_FREE(cache_table_);
        model_ptr_ = nullptr;
        cache_table_ = nullptr;

        #if KINEMATICS_ENABLE_DEBUG
            print("[KIN] Failed to allocate stats memory\r\n");
        #endif
        return false;
    }

    // 初始化统计
    memset(stats_ptr_, 0, sizeof(kin_perf_stats_t));

    // 预计算网格按需分配
    grid_ptr_ = nullptr;

    return true;
}

void kinematics::freeMemory() {
    if (model_ptr_) {
        KINEMATICS_FREE(model_ptr_);
        model_ptr_ = nullptr;
    }

    if (cache_table_) {
        KINEMATICS_FREE(cache_table_);
        cache_table_ = nullptr;
    }

    if (grid_ptr_) {
        if (grid_ptr_->data) {
            KINEMATICS_FREE(grid_ptr_->data);
        }
        KINEMATICS_FREE(grid_ptr_);
        grid_ptr_ = nullptr;
    }

    if (stats_ptr_) {
        KINEMATICS_FREE(stats_ptr_);
        stats_ptr_ = nullptr;
    }
}

void kinematics::clearCache() {
    if (!cache_table_) return;

    for (uint16_t i = 0; i < cache_size_; i++) {
        cache_table_[i].valid = false;
    }
    cache_index_ = 0;
}

// ==================== 模型配置 ====================

bool kinematics::setModel(float L1, float L2, float L3, float L4, float L5) {
    if (!model_ptr_) return false;

    // 验证参数
    if (L1 <= 0 || L2 <= 0 || L3 <= 0 || L4 <= 0 || L5 <= 0) {
        return false;
    }

    model_ptr_->L1 = L1;
    model_ptr_->L2 = L2;
    model_ptr_->L3 = L3;
    model_ptr_->L4 = L4;
    model_ptr_->L5 = L5;

    updateModelPrecompute();
    clearCache();

    #if KINEMATICS_ENABLE_DEBUG
        print("[KIN] Model set: L1=%.1f, L2=%.1f, L3=%.1f, L4=%.1f, L5=%.1f\r\n",
               L1, L2, L3, L4, L5);
    #endif

    return true;
}

void kinematics::updateModelPrecompute() {
    if (!model_ptr_) return;

    model_ptr_->L5_half = model_ptr_->L5 * 0.5f;

    model_ptr_->L1_sq = model_ptr_->L1 * model_ptr_->L1;
    model_ptr_->L2_sq = model_ptr_->L2 * model_ptr_->L2;
    model_ptr_->L3_sq = model_ptr_->L3 * model_ptr_->L3;
    model_ptr_->L4_sq = model_ptr_->L4 * model_ptr_->L4;
    model_ptr_->L5_sq = model_ptr_->L5 * model_ptr_->L5;

    model_ptr_->inv_2L1 = 1.0f / (2.0f * model_ptr_->L1);
    model_ptr_->inv_2L3 = 1.0f / (2.0f * model_ptr_->L3);
}

void kinematics::setWorkspace(float x_min, float x_max, float y_min, float y_max) {
    if (!model_ptr_) return;

    model_ptr_->workspace.x_min = x_min;
    model_ptr_->workspace.x_max = x_max;
    model_ptr_->workspace.y_min = y_min;
    model_ptr_->workspace.y_max = y_max;

    #if KINEMATICS_ENABLE_DEBUG
        print("[KIN] Workspace set: X=[%.1f, %.1f], Y=[%.1f, %.1f]\r\n",
               x_min, x_max, y_min, y_max);
    #endif
}

// ==================== 预计算变量 ====================

void kinematics::precomputeVars(float x, float y, CalcWorkspace& ws) const {
    if (!model_ptr_) return;

    // 输入
    ws.x = x;
    ws.y = y;

    // 计算平方值
    ws.vars.x_sq = x * x;
    ws.vars.y_sq = y * y;

    // 中间变量
    ws.vars.t = x - model_ptr_->L5;
    ws.vars.t_sq = ws.vars.t * ws.vars.t;

    // 使用快速数学函数
    if (fast_math_enabled_) {
        ws.vars.R1 = MathSqrtFast(ws.vars.t_sq + ws.vars.y_sq);
        ws.vars.R2 = MathSqrtFast(ws.vars.x_sq + ws.vars.y_sq);
    } else {
        ws.vars.R1 = sqrtf(ws.vars.t_sq + ws.vars.y_sq);
        ws.vars.R2 = sqrtf(ws.vars.x_sq + ws.vars.y_sq);
    }

    // 使用预计算的倒数
    ws.vars.K1 = (ws.vars.t_sq + ws.vars.y_sq + model_ptr_->L1_sq - model_ptr_->L2_sq)
                 * model_ptr_->inv_2L1;
    ws.vars.K2 = (ws.vars.x_sq + ws.vars.y_sq + model_ptr_->L3_sq - model_ptr_->L4_sq)
                 * model_ptr_->inv_2L3;

    // 基准角度
    ws.vars.phi_beta = MathAtan2(y, ws.vars.t);
    ws.vars.phi_alpha = MathAtan2(y, x);

    // 钳制参数
    ws.vars.arg_beta = MathClamp(ws.vars.K1 / ws.vars.R1, -1.0f, 1.0f);
    ws.vars.arg_alpha = MathClamp(ws.vars.K2 / ws.vars.R2, -1.0f, 1.0f);
}

// ==================== 核心计算 ====================

kin_ik_result_t kinematics::computeFromVars(const CalcWorkspace& ws) const {
    kin_ik_result_t result = {};

    // 检查解的存在性
    if (!checkSolutionExistence(ws.vars.arg_beta, ws.vars.arg_alpha)) {
        result.flags.error = 1;
        return result;
    }

    // 开始计时（使用FreeRTOS或系统计时器）
    #if KINEMATICS_USE_FREERTOS
        TickType_t start_ticks = xTaskGetTickCount();
    #elif defined(DWT) && defined(DWT_CYCCNT)
        uint32_t start_cycles = DWT->CYCCNT;
    #endif

    // 计算角度差值
    float delta_beta = MathAcos(ws.vars.arg_beta);
    float delta_alpha = MathAcos(ws.vars.arg_alpha);

    // 两个可能的解
    float beta1 = ws.vars.phi_beta + delta_beta;
    float beta2 = ws.vars.phi_beta - delta_beta;
    float alpha1 = ws.vars.phi_alpha + delta_alpha;
    float alpha2 = ws.vars.phi_alpha - delta_alpha;

    // 选择解
    result.alpha = (alpha1 >= MATH_PI_4) ? alpha1 : alpha2;
    result.beta = (beta1 <= MATH_PI_4) ? beta1 : beta2;

    // 归一化角度
    result.alpha = MathNormalizeRad2Pi(result.alpha);
    result.beta = MathNormalizeRad2Pi(result.beta);

    // 计算A点和C点
    kin_point2d_t A = calculatePointA(result.alpha);
    kin_point2d_t C = calculatePointC(result.beta);

    // 计算夹角
    float ABx = ws.vars.t - A.x;
    float ABy = ws.y - A.y;
    float CBx = ws.x - C.x;
    float CBy = ws.y - C.y;

    float magAB, magCB;
    if (fast_math_enabled_) {
        magAB = MathSqrtFast(ABx * ABx + ABy * ABy);
        magCB = MathSqrtFast(CBx * CBx + CBy * CBy);
    } else {
        magAB = sqrtf(ABx * ABx + ABy * ABy);
        magCB = sqrtf(CBx * CBx + CBy * CBy);
    }

    float cos_theta1 = MathClamp((A.x * ABx + A.y * ABy) / (model_ptr_->L3 * magAB), -1.0f, 1.0f);
    float cos_theta2 = MathClamp(((C.x - model_ptr_->L5) * CBx + C.y * CBy) / (model_ptr_->L1 * magCB), -1.0f, 1.0f);

    result.angles.theta1 = static_cast<uint16_t>(MathAcos(cos_theta1) * 100.0f);
    result.angles.theta2 = static_cast<uint16_t>(MathAcos(cos_theta2) * 100.0f);

    // 结束计时
    #if KINEMATICS_USE_FREERTOS
        TickType_t end_ticks = xTaskGetTickCount();
        result.compute_us = static_cast<uint16_t>((end_ticks - start_ticks) * 1000 / portTICK_PERIOD_MS);
    #elif defined(DWT) && defined(DWT_CYCCNT)
        uint32_t end_cycles = DWT->CYCCNT;
        result.compute_us = static_cast<uint16_t>(
            (end_cycles - start_cycles) * 1000000.0f / SystemCoreClock);
    #endif

    // 验证计算时间
    if (result.compute_us > KINEMATICS_MAX_COMPUTE_TIME_US) {
        #if KINEMATICS_ENABLE_DEBUG
            print("[KIN] Warning: Compute time exceeded threshold: %u us\r\n",
                   result.compute_us);
        #endif
    }

    result.flags.valid = 1;
    return result;
}

// ==================== 辅助函数 ====================

kin_point2d_t kinematics::calculatePointA(float alpha) const {
    kin_point2d_t A;
    if (fast_math_enabled_) {
        A.x = model_ptr_->L3 * MathCosFast(alpha);
        A.y = model_ptr_->L3 * MathSinFast(alpha);
    } else {
        A.x = model_ptr_->L3 * cosf(alpha);
        A.y = model_ptr_->L3 * sinf(alpha);
    }
    return A;
}

kin_point2d_t kinematics::calculatePointC(float beta) const {
    kin_point2d_t C;
    if (fast_math_enabled_) {
        C.x = model_ptr_->L5 + model_ptr_->L1 * MathCosFast(beta);
        C.y = model_ptr_->L1 * MathSinFast(beta);
    } else {
        C.x = model_ptr_->L5 + model_ptr_->L1 * cosf(beta);
        C.y = model_ptr_->L1 * sinf(beta);
    }
    return C;
}

bool kinematics::checkSolutionExistence(float arg_beta, float arg_alpha) const {
    return (fabsf(arg_beta) <= 1.0f + MATH_EPSILON) &&
           (fabsf(arg_alpha) <= 1.0f + MATH_EPSILON);
}

// ==================== 缓存管理 ====================

uint16_t kinematics::computeHash(float x, float y) const {
    // 简单哈希函数
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
            // 检查精度（1mm内认为相同）
            float dx = fabsf(entry.x - x);
            float dy = fabsf(entry.y - y);

            if (dx < 1.0f && dy < 1.0f) {
                result = entry.result;
                result.flags.cached = 1;
                result.compute_us = 0;  // 缓存命中不计时
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

// ==================== 主解算函数 ====================

kin_ik_result_t kinematics::inverseKinematics(float x, float y) {
    kin_ik_result_t result = {};

    if (!model_ptr_ || !stats_ptr_) {
        result.flags.error = 1;
        return result;
    }

    // 检查工作空间
    if (x < model_ptr_->workspace.x_min || x > model_ptr_->workspace.x_max ||
        y < model_ptr_->workspace.y_min || y > model_ptr_->workspace.y_max) {
        result.flags.error = 1;
        #if KINEMATICS_ENABLE_DEBUG
            print("[KIN] Point (%.1f, %.1f) outside workspace\r\n", x, y);
        #endif
        return result;
    }

    // 1. 检查缓存
    if (cache_enabled_ && getFromCache(x, y, result)) {
        updateStats(0, true, false);
        return result;
    }

    // 2. 网格插值（如果启用）
    if (grid_enabled_ && grid_ptr_ && grid_ptr_->enabled) {
        result = interpolateFromGrid(x, y);
        if (result.flags.valid) {
            updateStats(result.compute_us, false, false);
            return result;
        }
    }

    // 3. 快速路径检查（相邻点）
    if (checkFastPath(x, y, result)) {
        updateStats(0, false, true);
        return result;
    }

    // 4. 完整计算
    precomputeVars(x, y, workspace_);
    result = computeFromVars(workspace_);

    // 5. 更新缓存
    if (result.flags.valid && cache_enabled_) {
        addToCache(x, y, result);
    }

    // 6. 更新统计
    updateStats(result.compute_us, false, false);

    return result;
}

kin_ik_result_t kinematics::inverseKinematicsCached(float x, float y) {
    return inverseKinematics(x, y);
}

// ==================== 批量解算 ====================

uint32_t kinematics::inverseKinematicsBatch(const float* x_array, const float* y_array,
                                           kin_ik_result_t* results, uint32_t count) {
    if (!x_array || !y_array || !results || count == 0) {
        return 0;
    }

    uint32_t valid_count = 0;

    #if KINEMATICS_USE_FREERTOS
        // 批量计算时临时提高任务优先级
        UBaseType_t original_priority = uxTaskPriorityGet(NULL);
        vTaskPrioritySet(NULL, original_priority + 1);
    #endif

    for (uint32_t i = 0; i < count; i++) {
        results[i] = inverseKinematics(x_array[i], y_array[i]);
        if (results[i].flags.valid) {
            valid_count++;
        }
    }

    #if KINEMATICS_USE_FREERTOS
        // 恢复原始优先级
        vTaskPrioritySet(NULL, original_priority);
    #endif

    // 更新批处理统计
    if (stats_ptr_) {
        stats_ptr_->batch_calls++;
    }

    return valid_count;
}

// ==================== 正运动学 ====================

kin_fk_result_t kinematics::forwardKinematics(float alpha, float beta) {
    kin_fk_result_t result = {};
    result.solution_count = 0;
    result.valid = 0;

    if (!model_ptr_) return result;

    // 计算圆心
    kin_point2d_t C1 = calculatePointA(alpha);
    kin_point2d_t C2 = calculatePointC(beta);

    // 计算圆心距
    float dx = C2.x - C1.x;
    float dy = C2.y - C1.y;
    float d_sq = dx * dx + dy * dy;
    float d = sqrtf(d_sq);

    // 检查交点
    float r_sum = model_ptr_->L4 + model_ptr_->L2;
    float r_diff = fabsf(model_ptr_->L4 - model_ptr_->L2);

    if (d > r_sum || d < r_diff) {
        return result;
    }

    // 计算交点
    float a = (model_ptr_->L4_sq - model_ptr_->L2_sq + d_sq) / (2.0f * d);
    float h_sq = model_ptr_->L4_sq - a * a;

    if (h_sq < 0.0f) {
        return result;
    }

    float h = sqrtf(h_sq);

    // 中点
    float P2x = C1.x + (a * dx) / d;
    float P2y = C1.y + (a * dy) / d;

    // 两个交点
    result.solutions[0].x = P2x + (h * (-dy)) / d;
    result.solutions[0].y = P2y + (h * dx) / d;

    result.solutions[1].x = P2x - (h * (-dy)) / d;
    result.solutions[1].y = P2y - (h * dx) / d;

    result.solution_count = 2;
    result.valid = 1;

    return result;
}

// ==================== 快速路径检查 ====================

bool kinematics::checkFastPath(float x, float y, kin_ik_result_t& result) const {
    // 检查是否在上次计算点附近（2mm内）
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
        result.compute_us = 0;
        return true;
    }

    last_x_ = x;
    last_y_ = y;

    return false;
}

// ==================== 网格插值 ====================

kin_ik_result_t kinematics::interpolateFromGrid(float x, float y) const {
    if (!grid_enabled_ || !grid_ptr_ || !grid_ptr_->enabled || !grid_ptr_->data) {
        kin_ik_result_t result = {};
        return result;
    }

    // 检查是否在网格范围内
    if (x < grid_ptr_->x_min || x > grid_ptr_->x_max ||
        y < grid_ptr_->y_min || y > grid_ptr_->y_max) {
        kin_ik_result_t result = {};
        return result;
    }

    // 找到最近的网格点
    uint16_t i = static_cast<uint16_t>((y - grid_ptr_->y_min) / grid_ptr_->y_step);
    uint16_t j = static_cast<uint16_t>((x - grid_ptr_->x_min) / grid_ptr_->x_step);

    // 边界检查
    i = min<uint16_t>(i, grid_ptr_->grid_y - 1);
    j = min<uint16_t>(j, grid_ptr_->grid_x - 1);

    uint32_t index = i * grid_ptr_->grid_x + j;

    if (index < grid_ptr_->grid_x * grid_ptr_->grid_y) {
        kin_ik_result_t result = grid_ptr_->data[index];
        result.compute_us = 0;  // 网格插值不计时
        return result;
    }

    kin_ik_result_t result = {};
    return result;
}

// ==================== 性能统计 ====================

void kinematics::updateStats(uint16_t compute_time, bool cached, bool fast_path) {
    if (!stats_ptr_) return;

    stats_ptr_->total_calls++;

    if (cached) stats_ptr_->cache_hits++;
    if (fast_path) stats_ptr_->fast_path_hits++;

    // 更新计算时间统计
    if (compute_time > 0) {  // 只统计实际计算的时间
        if (stats_ptr_->total_calls == 1) {
            stats_ptr_->min_time_us = compute_time;
            stats_ptr_->max_time_us = compute_time;
            stats_ptr_->avg_time_us = compute_time;
        } else {
            if (compute_time < stats_ptr_->min_time_us) {
                stats_ptr_->min_time_us = compute_time;
            }
            if (compute_time > stats_ptr_->max_time_us) {
                stats_ptr_->max_time_us = compute_time;
            }

            // 移动平均
            stats_ptr_->avg_time_us = (stats_ptr_->avg_time_us * (stats_ptr_->total_calls - 1) +
                                      compute_time) / stats_ptr_->total_calls;
        }
    }

    // 更新命中率
    if (stats_ptr_->total_calls > 0) {
        stats_ptr_->cache_hit_rate =
            (stats_ptr_->cache_hits * 100.0f) / stats_ptr_->total_calls;
    }
}

kinematics::kin_perf_stats_t kinematics::getStats() const {
    if (stats_ptr_) {
        return *stats_ptr_;
    }
    return kin_perf_stats_t{};
}

void kinematics::resetStats() {
    if (stats_ptr_) {
        memset(stats_ptr_, 0, sizeof(kin_perf_stats_t));
        stats_ptr_->min_time_us = 0xFFFF;  // 初始化为最大值
    }

    // 重置快速路径状态
    fast_path_counter_ = 0;
    last_x_ = 0.0f;
    last_y_ = 0.0f;
    last_result_ = kin_ik_result_t{};
}

// ==================== FreeRTOS任务接口 ====================
#if KINEMATICS_USE_FREERTOS

bool kinematics::createTask(const char* task_name, UBaseType_t priority) {
    if (task_handle_ != nullptr) {
        return true;  // 任务已存在
    }

    // 创建消息队列
    msg_queue_ = xQueueCreate(KINEMATICS_QUEUE_SIZE, sizeof(kin_task_msg_t));
    if (msg_queue_ == nullptr) {
        #if KINEMATICS_ENABLE_DEBUG
            print("[KIN] Failed to create message queue\r\n");
        #endif
        return false;
    }

    // 创建完成信号量
    completion_sem_ = xSemaphoreCreateBinary();
    if (completion_sem_ == nullptr) {
        vQueueDelete(msg_queue_);
        msg_queue_ = nullptr;

        #if KINEMATICS_ENABLE_DEBUG
            print("[KIN] Failed to create completion semaphore\r\n");
        #endif
        return false;
    }

    // 创建任务
    BaseType_t result = xTaskCreate(
        taskFunction,
        task_name,
        KINEMATICS_TASK_STACK_SIZE / sizeof(StackType_t),
        nullptr,
        priority,
        &task_handle_
    );

    if (result != pdPASS) {
        vQueueDelete(msg_queue_);
        vSemaphoreDelete(completion_sem_);
        msg_queue_ = nullptr;
        completion_sem_ = nullptr;

        #if KINEMATICS_ENABLE_DEBUG
            print("[KIN] Failed to create task\r\n");
        #endif
        return false;
    }

    #if KINEMATICS_ENABLE_DEBUG
        print("[KIN] Task created: %s, priority: %lu\r\n",
               task_name, priority);
    #endif

    return true;
}

void kinematics::taskFunction(void* params) {
    (void)params;

    kin_task_msg_t msg;

    while (1) {
        // 等待消息
        if (xQueueReceive(msg_queue_, &msg, portMAX_DELAY) == pdTRUE) {
            if (task_instance_ == nullptr) {
                continue;
            }

            switch (msg.type) {
                case kin_task_msg_t::MSG_COMPUTE_IK: {
                    if (msg.ik_data.result_ptr != nullptr) {
                        *msg.ik_data.result_ptr = task_instance_->inverseKinematics(
                            msg.ik_data.x, msg.ik_data.y);
                    }
                    break;
                }

                case kin_task_msg_t::MSG_COMPUTE_BATCH: {
                    if (msg.batch_data.valid_count != nullptr) {
                        *msg.batch_data.valid_count = task_instance_->inverseKinematicsBatch(
                            msg.batch_data.x_array,
                            msg.batch_data.y_array,
                            msg.batch_data.results,
                            msg.batch_data.count);
                    }
                    break;
                }

                case kin_task_msg_t::MSG_UPDATE_MODEL: {
                    task_instance_->setModel(
                        msg.model_data.L[0],
                        msg.model_data.L[1],
                        msg.model_data.L[2],
                        msg.model_data.L[3],
                        msg.model_data.L[4]);
                    break;
                }

                case kin_task_msg_t::MSG_GET_STATS:
                    // 统计信息通过其他方式获取
                    break;
            }

            // 发送完成信号
            if (msg.completion_semaphore != nullptr) {
                xSemaphoreGive(msg.completion_semaphore);
            }
        }
    }
}

bool kinematics::requestInverseKinematics(float x, float y, kin_ik_result_t* result,
                                          TickType_t timeout) {
    if (msg_queue_ == nullptr || task_instance_ != this) {
        return false;
    }

    // 设置任务实例
    task_instance_ = this;

    kin_task_msg_t msg;
    msg.type = kin_task_msg_t::MSG_COMPUTE_IK;
    msg.ik_data.x = x;
    msg.ik_data.y = y;
    msg.ik_data.result_ptr = result;

    // 创建本地信号量用于同步
    msg.completion_semaphore = xSemaphoreCreateBinary();
    if (msg.completion_semaphore == nullptr) {
        return false;
    }

    // 发送消息
    if (xQueueSend(msg_queue_, &msg, timeout) != pdTRUE) {
        vSemaphoreDelete(msg.completion_semaphore);
        return false;
    }

    // 等待完成
    bool success = (xSemaphoreTake(msg.completion_semaphore, timeout) == pdTRUE);

    vSemaphoreDelete(msg.completion_semaphore);
    return success;
}

bool kinematics::requestBatchInverseKinematics(const float* x_array, const float* y_array,
                                               kin_ik_result_t* results, uint32_t count,
                                               uint32_t* valid_count, TickType_t timeout) {
    if (msg_queue_ == nullptr || task_instance_ != this) {
        return false;
    }

    task_instance_ = this;

    kin_task_msg_t msg;
    msg.type = kin_task_msg_t::MSG_COMPUTE_BATCH;
    msg.batch_data.x_array = x_array;
    msg.batch_data.y_array = y_array;
    msg.batch_data.results = results;
    msg.batch_data.count = count;
    msg.batch_data.valid_count = valid_count;

    msg.completion_semaphore = xSemaphoreCreateBinary();
    if (msg.completion_semaphore == nullptr) {
        return false;
    }

    if (xQueueSend(msg_queue_, &msg, timeout) != pdTRUE) {
        vSemaphoreDelete(msg.completion_semaphore);
        return false;
    }

    bool success = (xSemaphoreTake(msg.completion_semaphore, timeout) == pdTRUE);

    vSemaphoreDelete(msg.completion_semaphore);
    return success;
}

bool kinematics::waitForCompletion(TickType_t timeout) {
    if (completion_sem_ == nullptr) {
        return false;
    }
    return (xSemaphoreTake(completion_sem_, timeout) == pdTRUE);
}

#endif  // KINEMATICS_USE_FREERTOS

// ==================== 系统信息 ====================

const char* kinematics::getConfigInfo() {
    static char info[256];
    sprintf(info, "Kinematics Config: CCRAM=%d, FastMath=%d, Cache=%d, Grid=%d, FreeRTOS=%d, "
        "MathLevel=%d, CacheSize=%d, GridSize=%dx%d",
        KINEMATICS_USE_CCRAM, KINEMATICS_USE_FAST_MATH, KINEMATICS_CACHE_ENABLED,
        KINEMATICS_GRID_ENABLED, KINEMATICS_USE_FREERTOS, KINEMATICS_MATH_LEVEL,
        KINEMATICS_CACHE_SIZE, KINEMATICS_GRID_SIZE_X, KINEMATICS_GRID_SIZE_Y);
    return info;
}

const char* kinematics::getVersion() {
    return KinematicsVersion;
}

bool kinematics::isInitialized() const {
    return (model_ptr_ != nullptr && stats_ptr_ != nullptr);
}

bool kinematics::isCCRAMEnabled() const {
    return KINEMATICS_USE_CCRAM != 0;
}

void kinematics::getMemoryUsage(uint32_t* used, uint32_t* free) const {
    if (used) *used = 0;
    if (free) *free = 0;

    #if KINEMATICS_USE_CCRAM && MATH_USE_CCRAM
        size_t used_bytes, free_bytes;
        uint32_t fragments;
        CCRAM_GetStats(&used_bytes, &free_bytes, &fragments);

        if (used) *used = used_bytes;
        if (free) *free = free_bytes;
    #endif
}

void kinematics::enableCache(bool enable) {
    cache_enabled_ = enable && KINEMATICS_CACHE_ENABLED;
}

void kinematics::enableFastMath(bool enable) {
    fast_math_enabled_ = enable && KINEMATICS_USE_FAST_MATH;
}

void kinematics::enableGrid(bool enable) {
    grid_enabled_ = enable && KINEMATICS_GRID_ENABLED;
}
