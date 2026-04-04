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
// Created by Administrator on 2026/2/24.
//

#include "config.h"
#include "remote_task.h"
#include "rx_vtm.h"
#include "bsp_error_handler.h"
#include "vtm.h"

#ifdef USE_VTM

bsp::UART* vtm_uart = nullptr;
remote::VTM* vtm = nullptr;

void VIMChannelsInit(RemoteDeviceConfig_s* config) {
    if (!config)
        RM_ASSERT_TRUE(false, "VTM config is null");
    vtm_uart = new bsp::UART(&config->uart);
    vtm_uart->SetBaudrate(config->baudrate);
    vtm_uart->SetupRx(VTM_BUFFER_SIZE);
    vtm = new remote::VTM(vtm_uart);

    // 初始化rcData为中值
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcData[i] = 1500.0f;
    }
}

/**
 * @brief 简单直接的VTM数据同步到rcData
 * @note 最基础的通道映射功能
 */
void SyncVTMToRcData() {
    if (vtm == nullptr) return;

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

    // 键盘数据映射 [AUX17-AUX26] - 按位存储
    uint16_t keyCode = vtm->keyboard.code;
    for (int i = 0; i < 10; i++) {
        rcData[AUX13 + i] = static_cast<float>((keyCode >> i) & 0x1);
    }
}

#endif