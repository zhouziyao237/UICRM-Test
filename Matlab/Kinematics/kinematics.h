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

#pragma once

#include "main.h"
#include "arm_math.h"

class Kinematics {
public:
    // 机构参数结构体
    struct Model {
        float L1; // 右侧第一连杆长度
        float L2; // 右侧第二连杆长度
        float L3; // 左侧第一连杆长度
        float L4; // 左侧第二连杆长度
        float L5; // 左右基座间距
    };

    // 逆解结果结构体
    struct IKResult {
        float alpha; // 左侧关节角度（弧度）
        float beta;  // 右侧关节角度（弧度）
        float midpointToEndDistance; // 中间中点到足端的距离
        float midpointToEndAngle;    // 中间中点到足端的角度（弧度）
    };

    // 正解结果结构体
    struct FKResult {
        float x1; // 第一个交点X坐标
        float y1; // 第一个交点Y坐标
        float x2; // 第二个交点X坐标
        float y2; // 第二个交点Y坐标
        float alpha; // 左侧关节角度（弧度）
        float beta;  // 右侧关节角度（弧度）
        float midpointToEndDistance; // 中间中点到足端的距离
        float midpointToEndAngle;    // 中间中点到足端的角度（弧度）
        bool hasSolution; // 是否有解
    };

    // 构造函数
    Kinematics();
    
    // 设置机构参数
    void setModel(Model model);
    
    // 逆运动学计算：根据末端坐标计算关节角度
    IKResult inverseKinematics(float x, float y);
    
    // 正运动学计算：根据关节角度计算末端坐标
    FKResult forwardKinematics(float alpha, float beta);
    
private:
    Model model; // 机构参数
    
    // 计算中间中点到足端的距离和角度
    void calculateMidpointInfo(float x, float y, float &distance, float &angle);
};

