/* fivebar.cpp - 五连杆运动学实现 */
#include "fivebar.h"

#include <algorithm>
#include <cmath>

#include "bsp_os.h"
#include "bsp_print.h"

// ==================== 构造函数/析构函数 ====================

FiveBarLinkage::FiveBarLinkage() :
    model_initialized_(false)
    #if KIN_USE_GRID
    , grid_initialized_(false)
    , grid_enabled_(false)
    #endif
{
    // 设置默认模型参数
    model_.set(45.0f, 80.0f, 45.0f, 80.0f, 55.0f);
    model_initialized_ = true;

    #if KIN_USE_GRID
    // 初始化网格数据
    for (int i = 0; i < GRID_X * GRID_Y; ++i) {
        grid_valid_[i] = false;
        grid_data_[i] = kin_ik_result_t{};
    }
    #endif

    #if KIN_DEBUG
    print("[FIVEBAR] Five-bar linkage initialized\r\n");
    #endif
}

FiveBarLinkage::~FiveBarLinkage() {
    #if KIN_DEBUG
    print("[FIVEBAR] Five-bar linkage destroyed\r\n");
    #endif
}

FiveBarLinkage::FiveBarLinkage(FiveBarLinkage&& other) noexcept :
    kinematics(std::move(other)),
    model_(other.model_),
    model_initialized_(other.model_initialized_),
    calc_ws_(other.calc_ws_)
    #if KIN_USE_GRID
    , grid_initialized_(other.grid_initialized_)
    , grid_enabled_(other.grid_enabled_)
    #endif
{
    #if KIN_USE_GRID
    // 复制网格数据
    for (int i = 0; i < GRID_X * GRID_Y; ++i) {
        grid_data_[i] = other.grid_data_[i];
        grid_valid_[i] = other.grid_valid_[i];
    }
    #endif

    other.model_initialized_ = false;
    #if KIN_USE_GRID
    other.grid_initialized_ = false;
    #endif
}

FiveBarLinkage& FiveBarLinkage::operator=(FiveBarLinkage&& other) noexcept {
    if (this != &other) {
        kinematics::operator=(std::move(other));
        model_ = other.model_;
        model_initialized_ = other.model_initialized_;
        calc_ws_ = other.calc_ws_;

        #if KIN_USE_GRID
        grid_initialized_ = other.grid_initialized_;
        grid_enabled_ = other.grid_enabled_;

        // 复制网格数据
        for (int i = 0; i < GRID_X * GRID_Y; ++i) {
            grid_data_[i] = other.grid_data_[i];
            grid_valid_[i] = other.grid_valid_[i];
        }
        #endif

        other.model_initialized_ = false;
        #if KIN_USE_GRID
        other.grid_initialized_ = false;
        #endif
    }
    return *this;
}

// ==================== 模型配置 ====================

bool FiveBarLinkage::setModel(float L1, float L2, float L3, float L4, float L5) {
    // 验证参数
    if (L1 <= 0 || L2 <= 0 || L3 <= 0 || L4 <= 0 || L5 <= 0) {
        #if KIN_DEBUG
        print("[FIVEBAR] Invalid model parameters\r\n");
        #endif
        return false;
    }

    model_.set(L1, L2, L3, L4, L5);
    model_initialized_ = true;

    // 清空缓存（模型参数改变）
    clearCache();

    #if KIN_DEBUG
    print("[FIVEBAR] Model set: L1=%.1f, L2=%.1f, L3=%.1f, L4=%.1f, L5=%.1f\r\n",
           L1, L2, L3, L4, L5);
    #endif

    return true;
}

bool FiveBarLinkage::isInitialized() const {
    return model_initialized_;
}

// ==================== 核心算法 ====================

void FiveBarLinkage::precomputeVars(float x, float y, CalcWorkspace& ws) const {
    ws.x = x;
    ws.y = y;

    ws.vars.x_sq = x * x;
    ws.vars.y_sq = y * y;

    ws.vars.t = x - model_.L5;
    ws.vars.t_sq = ws.vars.t * ws.vars.t;

    // 使用快速数学函数（根据宏定义条件编译）
    #if KIN_USE_FAST
    ws.vars.R1 = fastSqrt(ws.vars.t_sq + ws.vars.y_sq);
    ws.vars.R2 = fastSqrt(ws.vars.x_sq + ws.vars.y_sq);
    #else
    ws.vars.R1 = sqrtf(ws.vars.t_sq + ws.vars.y_sq);
    ws.vars.R2 = sqrtf(ws.vars.x_sq + ws.vars.y_sq);
    #endif

    ws.vars.K1 = (ws.vars.t_sq + ws.vars.y_sq + model_.L1_sq - model_.L2_sq)
                 * model_.inv_2L1;
    ws.vars.K2 = (ws.vars.x_sq + ws.vars.y_sq + model_.L3_sq - model_.L4_sq)
                 * model_.inv_2L3;

    #if KIN_USE_FAST
    ws.vars.phi_beta = fastAtan2(y, ws.vars.t);
    ws.vars.phi_alpha = fastAtan2(y, x);
    #else
    ws.vars.phi_beta = atan2f(y, ws.vars.t);
    ws.vars.phi_alpha = atan2f(y, x);
    #endif

    ws.vars.arg_beta = clamp(ws.vars.K1 / ws.vars.R1, -1.0f, 1.0f);
    ws.vars.arg_alpha = clamp(ws.vars.K2 / ws.vars.R2, -1.0f, 1.0f);
}

kin_ik_result_t FiveBarLinkage::computeFromVars(const CalcWorkspace& ws) const {
    kin_ik_result_t result;

    // 检查解的存在性
    if (!checkSolutionExistence(ws.vars.arg_beta, ws.vars.arg_alpha)) {
        result.flags.error = 1;
        return result;
    }

    // 开始计时
    uint32_t start_time = bsp::GetHighresTickMicroSec();

    // 计算角度差值
    float delta_beta = acosf(ws.vars.arg_beta);
    float delta_alpha = acosf(ws.vars.arg_alpha);

    // 两个可能的解
    float beta1 = ws.vars.phi_beta + delta_beta;
    float beta2 = ws.vars.phi_beta - delta_beta;
    float alpha1 = ws.vars.phi_alpha + delta_alpha;
    float alpha2 = ws.vars.phi_alpha - delta_alpha;

    // 选择合理的解
    result.alpha = (alpha1 >= M_PI_4) ? alpha1 : alpha2;
    result.beta = (beta1 <= M_PI_4) ? beta1 : beta2;

    // 归一化角度
    result.alpha = normalizeAngle(result.alpha);
    result.beta = normalizeAngle(result.beta);

    // 计算A点和C点
    kin_point2d_t A = computePointA(result.alpha);
    kin_point2d_t C = computePointC(result.beta);

    // 计算夹角
    float ABx = ws.vars.t - A.x;
    float ABy = ws.y - A.y;
    float CBx = ws.x - C.x;
    float CBy = ws.y - C.y;

    #if KIN_USE_FAST
    float magAB = fastSqrt(ABx * ABx + ABy * ABy);
    float magCB = fastSqrt(CBx * CBx + CBy * CBy);
    #else
    float magAB = sqrtf(ABx * ABx + ABy * ABy);
    float magCB = sqrtf(CBx * CBx + CBy * CBy);
    #endif

    #if KIN_USE_FAST
    float cos_theta1 = clamp((A.x * ABx + A.y * ABy) / (model_.L3 * magAB), -1.0f, 1.0f);
    float cos_theta2 = clamp(((C.x - model_.L5) * CBx + C.y * CBy) / (model_.L1 * magCB), -1.0f, 1.0f);
    #else
    float cos_theta1 = clamp((A.x * ABx + A.y * ABy) / (model_.L3 * magAB), -1.0f, 1.0f);
    float cos_theta2 = clamp(((C.x - model_.L5) * CBx + C.y * CBy) / (model_.L1 * magCB), -1.0f, 1.0f);
    #endif

    result.angles.theta1 = static_cast<uint16_t>(acosf(cos_theta1) * 100.0f);
    result.angles.theta2 = static_cast<uint16_t>(acosf(cos_theta2) * 100.0f);

    // 结束计时
    result.compute_us = bsp::GetHighresTickMicroSec() - start_time;

    result.flags.valid = 1;
    return result;
}

bool FiveBarLinkage::checkSolutionExistence(float arg_beta, float arg_alpha) const {
    return (fabsf(arg_beta) <= 1.0f + 1e-6f) &&
           (fabsf(arg_alpha) <= 1.0f + 1e-6f);
}

// ==================== 几何计算 ====================

kin_point2d_t FiveBarLinkage::calculatePointA(float alpha) const {
    return computePointA(alpha);
}

kin_point2d_t FiveBarLinkage::calculatePointC(float beta) const {
    return computePointC(beta);
}

kin_point2d_t FiveBarLinkage::computePointA(float alpha) const {
    kin_point2d_t A;
    #if KIN_USE_FAST
    A.x = model_.L3 * fastCos(alpha);
    A.y = model_.L3 * fastSin(alpha);
    #else
    A.x = model_.L3 * cosf(alpha);
    A.y = model_.L3 * sinf(alpha);
    #endif
    return A;
}

kin_point2d_t FiveBarLinkage::computePointC(float beta) const {
    kin_point2d_t C;
    #if KIN_USE_FAST
    C.x = model_.L5 + model_.L1 * fastCos(beta);
    C.y = model_.L1 * fastSin(beta);
    #else
    C.x = model_.L5 + model_.L1 * cosf(beta);
    C.y = model_.L1 * sinf(beta);
    #endif
    return C;
}

// ==================== 主解算函数 ====================

kin_ik_result_t FiveBarLinkage::inverseKinematics(float x, float y) {
    kin_ik_result_t result;

    if (!model_initialized_) {
        result.flags.error = 1;
        return result;
    }

    // 1. 检查工作空间
    if (!inWorkspace(x, y)) {
        result.flags.error = 1;
        #if KIN_DEBUG
        print("[FIVEBAR] Point (%.1f, %.1f) outside workspace\r\n", x, y);
        #endif
        updateStats(0, false, false);
        return result;
    }

    // 2. 快速路径检查
    if (checkFastPath(x, y, result)) {
        updateStats(0, false, true);
        return result;
    }

    // 3. 检查缓存
    if (cache_enabled_ && getFromCache(x, y, result)) {
        updateStats(0, true, false);
        return result;
    }

    // 4. 网格插值（如果启用）
    #if KIN_USE_GRID
    if (grid_enabled_) {
        result = interpolateFromGrid(x, y);
        if (result.flags.valid) {
            updateStats(result.compute_us, false, false);
            return result;
        }
    }
    #endif

    // 5. 完整计算
    precomputeVars(x, y, calc_ws_);
    result = computeFromVars(calc_ws_);

    // 6. 更新缓存
    if (result.flags.valid && cache_enabled_) {
        addToCache(x, y, result);
    }

    // 7. 更新统计和最后结果
    updateStats(result.compute_us, false, false);
    setLastResult(x, y, result);

    return result;
}

kin_ik_result_t FiveBarLinkage::inverseKinematicsCached(float x, float y) {
    return inverseKinematics(x, y);
}

// ==================== 正运动学 ====================

kin_fk_result_t FiveBarLinkage::forwardKinematics(float alpha, float beta) {
    kin_fk_result_t result;

    if (!model_initialized_) {
        result.valid = 0;
        return result;
    }

    // 计算圆心
    kin_point2d_t C1 = computePointA(alpha);
    kin_point2d_t C2 = computePointC(beta);

    // 计算圆心距
    float dx = C2.x - C1.x;
    float dy = C2.y - C1.y;
    float d_sq = dx * dx + dy * dy;
    float d = sqrtf(d_sq);

    // 检查交点
    float r_sum = model_.L4 + model_.L2;
    float r_diff = fabsf(model_.L4 - model_.L2);

    if (d > r_sum || d < r_diff) {
        result.valid = 0;
        return result;
    }

    // 计算交点
    float a = (model_.L4_sq - model_.L2_sq + d_sq) / (2.0f * d);
    float h_sq = model_.L4_sq - a * a;

    if (h_sq < 0.0f) {
        result.valid = 0;
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

// ==================== 缓存管理 ====================

void FiveBarLinkage::clearCache() {
    kinematics::clearCache();

    // 五连杆特有的缓存清理
    fast_path_counter_ = 0;
    last_x_ = last_y_ = 0.0f;
    last_result_ = kin_ik_result_t();

    #if KIN_USE_GRID
    // 清空网格数据
    for (int i = 0; i < GRID_X * GRID_Y; ++i) {
        grid_valid_[i] = false;
    }
    grid_initialized_ = false;
    #endif
}

// ==================== 网格插值 ====================
#if KIN_USE_GRID

kin_ik_result_t FiveBarLinkage::interpolateFromGrid(float x, float y) const {
    // 检查网格是否已初始化
    if (!grid_initialized_) {
        kin_ik_result_t result = {};
        result.flags.error = 1;
        return result;
    }

    // 计算网格步长
    float x_step = (workspace_.x_max - workspace_.x_min) / static_cast<float>(GRID_X);
    float y_step = (workspace_.y_max - workspace_.y_min) / static_cast<float>(GRID_Y);

    // 计算网格索引
    int x_idx = static_cast<int>((x - workspace_.x_min) / x_step);
    int y_idx = static_cast<int>((y - workspace_.y_min) / y_step);

    // 边界检查
    if (x_idx < 0 || x_idx >= GRID_X - 1 || y_idx < 0 || y_idx >= GRID_Y - 1) {
        kin_ik_result_t result = {};
        result.flags.error = 1;
        return result;
    }

    // 获取四个邻近网格点的数据
    int idx00 = y_idx * GRID_X + x_idx;
    int idx01 = y_idx * GRID_X + (x_idx + 1);
    int idx10 = (y_idx + 1) * GRID_X + x_idx;
    int idx11 = (y_idx + 1) * GRID_X + (x_idx + 1);

    // 检查四个点是否都有有效数据
    if (!grid_valid_[idx00] || !grid_valid_[idx01] ||
        !grid_valid_[idx10] || !grid_valid_[idx11]) {
        kin_ik_result_t result = {};
        result.flags.error = 1;
        return result;
    }

    // 双线性插值
    float x_ratio = ((x - workspace_.x_min) - x_idx * x_step) / x_step;
    float y_ratio = ((y - workspace_.y_min) - y_idx * y_step) / y_step;

    // 插值alpha角度
    float alpha00 = grid_data_[idx00].alpha;
    float alpha01 = grid_data_[idx01].alpha;
    float alpha10 = grid_data_[idx10].alpha;
    float alpha11 = grid_data_[idx11].alpha;

    float alpha_top = alpha00 * (1.0f - x_ratio) + alpha01 * x_ratio;
    float alpha_bottom = alpha10 * (1.0f - x_ratio) + alpha11 * x_ratio;
    float alpha_interp = alpha_top * (1.0f - y_ratio) + alpha_bottom * y_ratio;

    // 插值beta角度
    float beta00 = grid_data_[idx00].beta;
    float beta01 = grid_data_[idx01].beta;
    float beta10 = grid_data_[idx10].beta;
    float beta11 = grid_data_[idx11].beta;

    float beta_top = beta00 * (1.0f - x_ratio) + beta01 * x_ratio;
    float beta_bottom = beta10 * (1.0f - x_ratio) + beta11 * x_ratio;
    float beta_interp = beta_top * (1.0f - y_ratio) + beta_bottom * y_ratio;

    // 构建结果
    kin_ik_result_t result = {};
    result.alpha = alpha_interp;
    result.beta = beta_interp;

    // 插值角度
    result.angles.theta1 = static_cast<uint16_t>(
        grid_data_[idx00].angles.theta1 * (1.0f - x_ratio) * (1.0f - y_ratio) +
        grid_data_[idx01].angles.theta1 * x_ratio * (1.0f - y_ratio) +
        grid_data_[idx10].angles.theta1 * (1.0f - x_ratio) * y_ratio +
        grid_data_[idx11].angles.theta1 * x_ratio * y_ratio
    );

    result.angles.theta2 = static_cast<uint16_t>(
        grid_data_[idx00].angles.theta2 * (1.0f - x_ratio) * (1.0f - y_ratio) +
        grid_data_[idx01].angles.theta2 * x_ratio * (1.0f - y_ratio) +
        grid_data_[idx10].angles.theta2 * (1.0f - x_ratio) * y_ratio +
        grid_data_[idx11].angles.theta2 * x_ratio * y_ratio
    );

    result.flags.valid = 1;
    result.flags.cached = 1;  // 标记为插值结果
    result.compute_us = 0;    // 插值计算时间忽略不计

    return result;
}

#endif
