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

#include "main.h"

#include "Kinematics.h"
#include "arm_math.h"
#include "bsp_print.h"
#include "cmsis_os.h"

static Kinematics kinematics;

void RM_RTOS_Init(void) {
    print_use_uart(&huart1, true, 921600);

    // 设置机构参数
    Kinematics::Model model;
    model.L1 = 45.0f;
    model.L2 = 80.0f;
    model.L3 = 45.0f;
    model.L4 = 80.0f;
    model.L5 = 55.0f;
    kinematics.setModel(model);
}
void RM_RTOS_Threads_Init(void) {
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);

    set_cursor(0, 0);
    clear_screen();

    // 逆运动学计算（根据足端坐标计算关节角度）
    float x = 100.0f;  // 足端X坐标
    float y = 50.0f;   // 足端Y坐标
    Kinematics::IKResult ikResult = kinematics.inverseKinematics(x, y);
    print("逆解结果：\r\n");
    print("alpha: %.4f rad (%.1f°)\r\n", ikResult.alpha, ikResult.alpha * 180.0f / PI);
    print("beta: %.4f rad (%.1f°)\r\n", ikResult.beta, ikResult.beta * 180.0f / PI);
    print("中点到足端距离: %.4f mm\r\n", ikResult.midpointToEndDistance);
    print("中点到足端角度: %.4f rad (%.1f°)\r\n", ikResult.midpointToEndAngle,
          ikResult.midpointToEndAngle * 180.0f / PI);

    // 正运动学计算（根据关节角度计算足端坐标）
    float alpha = ikResult.alpha;  // 左侧关节角度
    float beta = ikResult.beta;    // 右侧关节角度
    Kinematics::FKResult fkResult = kinematics.forwardKinematics(alpha, beta);
    if (fkResult.hasSolution) {
        print("正解结果：\r\n");
        print("交点1: (%.4f, %.4f)\r\n", fkResult.x1, fkResult.y1);
        print("交点2: (%.4f, %.4f)\r\n", fkResult.x2, fkResult.y2);
        print("中点到足端距离: %.4f mm\r\n", fkResult.midpointToEndDistance);
        print("中点到足端角度: %.4f rad (%.1f°)\r\n", fkResult.midpointToEndAngle,
              fkResult.midpointToEndAngle * 180.0f / PI);
    } else {
        print("正解无可行解\n");
    }
}