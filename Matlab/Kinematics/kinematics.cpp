/*###########################################################
# Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "Kinematics.h"

#include "main.h"
#include "arm_math.h"

Kinematics::Kinematics() {
    // 默认机构参数
    model.L1 = 45.0f;
    model.L2 = 80.0f;
    model.L3 = 45.0f;
    model.L4 = 80.0f;
    model.L5 = 55.0f;
}

void Kinematics::setModel(Model model) {
    this->model = model;
}

Kinematics::IKResult Kinematics::inverseKinematics(float x, float y) {
    IKResult result;
    float alpha_tmp[2] = {0.0f, 0.0f};
    float beta_tmp[2] = {0.0f, 0.0f};
    float t = x - model.L5;

    // 计算K1和K2
    float K2 = (x*x + y*y + model.L3*model.L3 - model.L4*model.L4) / (2.0f * model.L3);
    float K1 = (t*t + y*y + model.L1*model.L1 - model.L2*model.L2) / (2.0f * model.L1);

    // 计算beta角度
    float denominator_beta = sqrt(t*t + y*y);
    float arg_beta = K1 / denominator_beta;
    // 限制arg_beta在[-1, 1]范围内
    if (arg_beta > 1.0f) arg_beta = 1.0f;
    if (arg_beta < -1.0f) arg_beta = -1.0f;
    
    beta_tmp[0] = atan2(y, t) + acos(arg_beta);
    beta_tmp[1] = atan2(y, t) - acos(arg_beta);

    // 计算alpha角度
    float denominator_alpha = sqrt(x*x + y*y);
    float arg_alpha = K2 / denominator_alpha;
    // 限制arg_alpha在[-1, 1]范围内
    if (arg_alpha > 1.0f) arg_alpha = 1.0f;
    if (arg_alpha < -1.0f) arg_alpha = -1.0f;
    
    alpha_tmp[0] = atan2(y, x) + acos(arg_alpha);
    alpha_tmp[1] = atan2(y, x) - acos(arg_alpha);

    // 角度归一化到[0, 2*PI]
    alpha_tmp[0] = fmod(alpha_tmp[0], 2*PI);
    if (alpha_tmp[0] < 0) alpha_tmp[0] += 2*PI;
    
    alpha_tmp[1] = fmod(alpha_tmp[1], 2*PI);
    if (alpha_tmp[1] < 0) alpha_tmp[1] += 2*PI;
    
    beta_tmp[0] = fmod(beta_tmp[0], 2*PI);
    if (beta_tmp[0] < 0) beta_tmp[0] += 2*PI;
    
    beta_tmp[1] = fmod(beta_tmp[1], 2*PI);
    if (beta_tmp[1] < 0) beta_tmp[1] += 2*PI;

    // 选择合适的角度解
    if (alpha_tmp[0] >= PI/4) {
        result.alpha = alpha_tmp[0];
    } else {
        result.alpha = alpha_tmp[1];
    }
    
    if (beta_tmp[0] <= PI/4) {
        result.beta = beta_tmp[0];
    } else {
        result.beta = beta_tmp[1];
    }

    // 计算中间中点到足端的距离和角度
    calculateMidpointInfo(x, y, result.midpointToEndDistance, result.midpointToEndAngle);

    return result;
}

Kinematics::FKResult Kinematics::forwardKinematics(float alpha, float beta) {
    FKResult result;
    result.hasSolution = false;

    // 计算圆心坐标
    float C1x = model.L3 * cos(alpha);
    float C1y = model.L3 * sin(alpha);
    
    float C2x = model.L5 + model.L1 * cos(beta);
    float C2y = model.L1 * sin(beta);

    // 计算圆心距
    float dx = C2x - C1x;
    float dy = C2y - C1y;
    float d = sqrt(dx*dx + dy*dy);

    // 检查解的存在性
    if (d <= model.L4 + model.L2 && d >= fabs(model.L4 - model.L2)) {
        // 计算交点
        float a = (model.L4*model.L4 - model.L2*model.L2 + d*d) / (2.0f * d);
        float h = sqrt(model.L4*model.L4 - a*a);
        
        // 中点坐标
        float P2x = C1x + (a * dx) / d;
        float P2y = C1y + (a * dy) / d;
        
        // 计算两个交点
        result.x1 = P2x + (h * (-dy)) / d;
        result.y1 = P2y + (h * dx) / d;
        
        result.x2 = P2x - (h * (-dy)) / d;
        result.y2 = P2y - (h * dx) / d;
        
        // 保存输入的角度
        result.alpha = alpha;
        result.beta = beta;
        
        // 计算中间中点到足端的距离和角度（使用第一个交点）
        calculateMidpointInfo(result.x1, result.y1, result.midpointToEndDistance, result.midpointToEndAngle);
        
        result.hasSolution = true;
    }

    return result;
}

void Kinematics::calculateMidpointInfo(float x, float y, float &distance, float &angle) {
    // 计算中间中点坐标（左右基座的中点）
    float midpointX = model.L5 / 2.0f;
    float midpointY = 0.0f;
    
    // 计算中点到足端的距离
    float dx = x - midpointX;
    float dy = y - midpointY;
    distance = sqrt(dx*dx + dy*dy);
    
    // 计算中点到足端的角度（相对于X轴正方向）
    angle = atan2(dy, dx);
    // 确保角度为正
    if (angle < 0) angle += 2*PI;
}

