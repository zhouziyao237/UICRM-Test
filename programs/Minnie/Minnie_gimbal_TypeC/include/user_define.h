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

// todo:舵机引脚为PE9
#define MG995_htim htim8
#define MG995_channel 2

#define WAIT_CHASSIS_ONLINE_OS_DELAY 5
#define PROTECT_OS_DELAY 1
#define SHOOT_OS_DELAY 1
#define SHOOT_SINGLE_OS_DELAY 1
#define CHASSIS_OS_DELAY 5
#define GIMBAL_OS_DELAY 1
#define REMOTE_OS_DELAY 1
#define DETECT_OS_DELAY 30
#define UI_OS_DELAY 100
#define SHOOT_REFEREE 0
#define ENABLE_UI 1

#define Debug_true 1
#define Debug_false 0

#define Control_true 1
#define Control_false 0

#define UART_PRINT_LOG

// 过低就会直接导致在pitch轴上过2PI了, 值在边界就会抖超过边界
#define pitch_bisa -0.6
#define yaw_bisa -3.0

#define singleShotDivider 3.5f

#define REMOTE_MODE_OS_DELAY 0.5

#define referee_uart_post huart1
#define refereerc_uart_post huart1
#define debug_uart_post huart6

#define dbus_uart_post huart3
#define sbus_uart_post huart3

#define GINBAL_DEBUG
// #define AI_AUTO_MODE
// #define SHOOT_REAT_CONTROL

// #define SBUS_MODE
#define DBUS_MODE

inline bool protect_wraning_flag = true;

typedef struct {
    float x;
    float y;
    float z;
} INS_Position_t;
typedef struct {
    float pitch;
    float roll;
    float yaw;
} INS_Angle_t;