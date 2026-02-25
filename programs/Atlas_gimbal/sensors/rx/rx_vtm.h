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

#pragma once

#include "config.h"
#include "vtm.h"

#ifdef USE_VTM
extern bsp::UART* vtm_uart;
extern remote::VTM* vtm;

/**
 * @brief 同步VTM数据到rcData通道
 * @note 简单直接的通道映射，无额外配置
 */
void SyncVTMToRcData();

/**
 * @brief 初始化VTM通道
 */
void VIMChannelsInit(RemoteDeviceConfig_s* config);

#endif

