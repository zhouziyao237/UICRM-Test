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

namespace remote {
    // VTM鼠标数据结构
    typedef struct {
        int16_t x;  // 鼠标左右移动速度 (负值向左)
        int16_t y;  // 鼠标前后移动速度 (负值向后)
        int16_t z;  // 鼠标滚轮速度 (负值向后)
        uint8_t l : 2;   // 左键: 0=未按下, 1=按下
        uint8_t r : 2;   // 右键: 0=未按下, 1=按下
        uint8_t m : 2;   // 中键: 0=未按下, 1=按下
    } __packed vtm_mouse_t;

#define vtm_mouse_xy_max 32767.0

    // VTM键盘数据结构
    typedef union {
        uint16_t code;
        struct {
            uint16_t W : 1;
            uint16_t S : 1;
            uint16_t A : 1;
            uint16_t D : 1;
            uint16_t SHIFT : 1;
            uint16_t CTRL : 1;
            uint16_t Q : 1;
            uint16_t E : 1;
            uint16_t R : 1;
            uint16_t F : 1;
            uint16_t G : 1;
            uint16_t Z : 1;
            uint16_t X : 1;
            uint16_t C : 1;
            uint16_t V : 1;
            uint16_t B : 1;
        } __packed bit;
    } __packed vtm_keyboard_t;

}  // namespace remote