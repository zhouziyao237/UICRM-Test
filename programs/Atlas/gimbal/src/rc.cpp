// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by Administrator on 2026/2/28.
//

#include "rc.h"

// ----------------------------- RC Task code -----------------------------

 osThreadId_t RCTaskHandle;

// ----------------------------- RC Data Code -----------------------------

float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT] = {0};
remote::VTM* vtm = nullptr;

// 配置按键
rc_key_config_t rc_keys[RCKEYSNUM] = CONFIGS;

// ----------------------------- RC Main Code -----------------------------

void InitRC() {
    if (!rc_uart)
        RM_ASSERT_TRUE(false, "RC UART init failed!");

    vtm = new remote::VTM(rc_uart);

    for (int i = 0; i < RCKEYSNUM; i++) {
        rc_key_add(rc_keys[i].channel, rc_keys[i]);
    }
}

void RestartRC(int stage) {
    switch(stage) {
        case 0:  // 主要逻辑：执行所有复位
            // 直接包含所有复位操作
            for (int i = 0; i < 6; i++) rcData[i] = 0.0f;   // 摇杆
            for (int i = 6; i < 12; i++) rcData[i] = 0;        // 开关
            for (int i = 12; i < 18; i++) rcData[i] = 0;       // 鼠标
            for (int i = 18; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) rcData[i] = 0; // 键盘
            break;

        case 1:  // 子功能：只复位摇杆
            for (int i = 0; i < 6; i++) rcData[i] = 1500.0f;
            break;

        case 2:  // 子功能：只复位开关按键
            for (int i = 6; i < 12; i++) rcData[i] = 0;
            break;

        default:
            // 处理无效参数
            break;
    }
}

void RCTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (!vtm) {
            RM_ASSERT_TRUE(false, "VTM init failed!\r\n");
        }

        if (!vtm->IsOnline()) {
            RestartRC();
            osDelay(1);
            continue;
        }

        // 摇杆通道映射 [AUX1-AUX6]
        rcData[ROLL] = static_cast<float>(vtm->ch0);  // 右摇杆X
        rcData[PITCH] = static_cast<float>(vtm->ch1);  // 右摇杆Y
        rcData[YAW] = static_cast<float>(vtm->ch2);  // 左摇杆Y
        rcData[THROTTLE] = static_cast<float>(vtm->ch3);  // 左摇杆X
        rcData[AUX1] = static_cast<float>(vtm->ch4);  // 拨轮
        rcData[AUX2] = static_cast<float>(vtm->ch5);  // 扳机键

        // 挡位开关映射 [AUX7]
        // VTM: C=0, N=1, S=2
        static const int gear_map[3] = {1, 2, 3};
        int gear_index = static_cast<int>(vtm->gear_switch);
        rcData[AUX3] = (gear_index >= 0 && gear_index < 3) ?
            static_cast<float>(gear_map[gear_index]) : 3.0f;

        // 按键映射 [AUX8-AUX10] - 存储0/1值
        rcData[AUX4] = vtm->pause_button ? 1.0f : 0.0f;    // 暂停按钮
        rcData[AUX5] = vtm->custom_left ? 1.0f : 0.0f;     // 自定义左键
        rcData[AUX6] = vtm->custom_right ? 1.0f : 0.0f;   // 自定义右键

        // 鼠标数据映射 [AUX11-AUX16]
        rcData[AUX7] = static_cast<float>(vtm->mouse.x);  // 鼠标X轴
        rcData[AUX8] = static_cast<float>(vtm->mouse.y);  // 鼠标Y轴
        rcData[AUX9] = static_cast<float>(vtm->mouse.z);  // 鼠标滚轮
        rcData[AUX10] = static_cast<float>(vtm->mouse.l);  // 鼠标左键
        rcData[AUX11] = static_cast<float>(vtm->mouse.r);  // 鼠标右键
        rcData[AUX12] = static_cast<float>(vtm->mouse.m);  // 鼠标中键

        // 键盘数据映射 [AUX13-AUX30] - 按位存储
        uint16_t keyCode = vtm->keyboard.code;
        for (int i = 0; i <16; i++) {
            rcData[AUX13 + i] = static_cast<float>((keyCode >> i) & 0x1);
        }

        for (int i = 0; i < RCKEYSNUM; i++) {
            rc_key_process(rc_keys[i].channel, (uint16_t)rcData[rc_keys[i].channel]);
        }
        // 处理待触发事件
        rc_key_process_events();

        osDelay(1);
    }
}

uint8_t RCIsOnline() {
    RobotStatus.RCOnline = (uint8_t)vtm->IsOnline();
    return RobotStatus.RCOnline;
}