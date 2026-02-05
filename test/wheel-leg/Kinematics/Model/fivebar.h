/* fivebar.h - 五连杆运动学模型 */
#pragma once

#include "../kinematics.h"

// 网格插值参数定义 - 根据宏定义条件编译
#if KIN_USE_GRID
#ifndef GRID_X
#define GRID_X 8  // 减少网格大小
#endif

#ifndef GRID_Y
#define GRID_Y 8  // 减少网格大小
#endif
#endif

class FiveBarLinkage : public kinematics {
public:
    // 构造函数/析构函数
    FiveBarLinkage();
    virtual ~FiveBarLinkage();

    // 移动构造函数和赋值运算符
    FiveBarLinkage(FiveBarLinkage&& other) noexcept;
    FiveBarLinkage& operator=(FiveBarLinkage&& other) noexcept;

    // ========== 实现基类接口 ==========
    kin_ik_result_t inverseKinematics(float x, float y) override;
    kin_fk_result_t forwardKinematics(float alpha, float beta) override;
    bool setModel(float L1, float L2, float L3, float L4, float L5) override;
    bool isInitialized() const override;

    // ========== 五连杆特有方法 ==========
    kin_ik_result_t inverseKinematicsCached(float x, float y);
    kin_point2d_t calculatePointA(float alpha) const;
    kin_point2d_t calculatePointC(float beta) const;

    // ========== 重写缓存方法 ==========
    void clearCache() override;

    // ========== 成员变量 ==========

    struct ModelParams {
        float L1, L2, L3, L4, L5;
        float L5_half;
        float L1_sq, L2_sq, L3_sq, L4_sq, L5_sq;
        float inv_2L1, inv_2L3;

        ModelParams() : L1(45), L2(80), L3(45), L4(80), L5(55) {
            update();
        }

        void set(float l1, float l2, float l3, float l4, float l5) {
            L1 = l1; L2 = l2; L3 = l3; L4 = l4; L5 = l5;
            update();
        }

        void update() {
            L5_half = L5 * 0.5f;
            L1_sq = L1 * L1;
            L2_sq = L2 * L2;
            L3_sq = L3 * L3;
            L4_sq = L4 * L4;
            L5_sq = L5 * L5;
            inv_2L1 = 1.0f / (2.0f * L1);
            inv_2L3 = 1.0f / (2.0f * L3);
        }
    } __attribute__((aligned(4)));

    ModelParams model_;

private:
    // ========== 私有类型定义 ==========

    // 计算工作区 - 使用更紧凑的设计
    struct CalcWorkspace {
        float x, y;
        struct {
            float x_sq, y_sq;
            float t, t_sq;
            float K1, K2;
            float R1, R2;
            float phi_beta, phi_alpha;
            float arg_beta, arg_alpha;
        } vars;
    } __attribute__((aligned(4)));

    // ========== 成员变量 ==========
    bool model_initialized_;

    // 计算工作区（避免重复分配）
    mutable CalcWorkspace calc_ws_;

    // 网格插值相关成员 - 根据宏定义条件编译
    #if KIN_USE_GRID
    bool grid_initialized_;
    kin_ik_result_t grid_data_[GRID_X * GRID_Y];
    bool grid_valid_[GRID_X * GRID_Y];
    bool grid_enabled_;
    #endif

    // ========== 私有辅助方法 ==========
    void precomputeVars(float x, float y, CalcWorkspace& ws) const;
    kin_ik_result_t computeFromVars(const CalcWorkspace& ws) const;
    bool checkSolutionExistence(float arg_beta, float arg_alpha) const;

    // 几何计算
    kin_point2d_t computePointA(float alpha) const;
    kin_point2d_t computePointC(float beta) const;

    // 网格插值（如果启用）
    #if KIN_USE_GRID
    kin_ik_result_t interpolateFromGrid(float x, float y) const;
    #endif
};
