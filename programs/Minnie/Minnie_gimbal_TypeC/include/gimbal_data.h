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
#include "gimbal.h"
#include "pid.h"

/**
 * @brief 云台初始化数据结构体实例
 *
 * 定义了云台系统的初始配置参数，包括偏移量、运动范围限制、方向设置等关键参数
 * 用于在系统启动时初始化云台控制模块的状态
 */
const control::gimbal_data_t gimbal_init_data = {
    .pitch_offset_ = pitch_bisa,      // 俯仰轴偏移量，使用预定义的俯仰偏差值
    .yaw_offset_ = yaw_bisa,          // 偏航轴偏移量，使用预定义的偏航偏差值
    .pitch_max_ = 0.45f,              // 俯仰轴最大运动角度限制（弧度）
    .yaw_max_ = PI,                   // 偏航轴最大运动角度限制（弧度）
    .yaw_circle_ = true,               // 偏航轴是否支持连续旋转功能
    .pitch_inverted = true,           // 俯仰轴运动方向是否反转
    .yaw_inverted = false,              // 偏航轴运动方向是否反转
    .pitch_eposition = 0,              // 俯仰轴期望位置初始值
    .yaw_eposition = 0,              // 偏航轴期望位置初始值
};
