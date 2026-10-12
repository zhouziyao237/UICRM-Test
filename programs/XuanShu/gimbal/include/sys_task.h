#pragma once

#include <cstdint>

#include "bsp_thread.h"
#include "rc_task.h"

struct AppStateSnapshot_t {
    GimbalMode mode = GimbalMode::KILL;
    bool rearm_required = true;
    uint32_t rc_generation = 0U;
    uint32_t generation = 0U;
};

void InitAppState();
void SysTask(void* arg);
AppStateSnapshot_t AppStateGetSnapshot();
bool AppIsKilled();

extern osThreadId_t SysTaskHandle;
inline const osThreadAttr_t SysTaskAttribute = {
    .name = "XuanShuSys",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh2,
    .tz_module = 0,
    .reserved = 0,
};
