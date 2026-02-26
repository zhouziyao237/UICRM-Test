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

#include "remote_task.h"

#include <cmath>

#include "bsp_print.h"
#include "rx_vtm.h"

// 全局rcData数组定义
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT] = {0};
RemoteDeviceConfig_t* RemoteDeviceConfig = nullptr;

osThreadId_t RemoteTaskHandle;

// ---------------------------------------- Model Main Code ----------------------------------------

void InitRemote() {
    RemoteDeviceConfig = new RemoteDeviceConfig_t{
        .uart = USE_VTM_UART,
        .baudrate = USE_VTM_BANDRATE,
    };
#ifdef USE_VTM
    VIMChannelsInit(RemoteDeviceConfig);
#endif

}

void RemoteMainTask(void* arg) {
    UNUSED(arg);
    while (true) {

#ifdef USE_VTM
        SyncVTMToRcData();
#endif

        osDelay(10);
    }
}