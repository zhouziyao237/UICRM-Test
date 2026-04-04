#pragma once

#include "bsp_thread.h"

constexpr uint32_t UI_OS_DELAY = 40;

void InitUI();
void UITask(void* arg);

extern osThreadId_t UITaskHandle;
inline const osThreadAttr_t UITaskAttribute = {
    .name = "UITask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
    .tz_module = 0,
    .reserved = 0,
};
