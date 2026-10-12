#pragma once

#include <cstdint>

#include "bsp_thread.h"

enum class GimbalMode : uint8_t {
    KILL = 0U,
    MANUAL,
    AUTOAIM,
};

struct RcSnapshot_t {
    bool online = false;
    bool physical_kill = true;
    GimbalMode requested_mode = GimbalMode::KILL;
    float pitch_rate_rad_s = 0.0f;
    float yaw_rate_rad_s = 0.0f;
    uint32_t generation = 0U;
};

void InitRC();
void RCTask(void* arg);
RcSnapshot_t RCGetSnapshot();
const char* GimbalModeName(GimbalMode mode);

extern osThreadId_t RCTaskHandle;
inline const osThreadAttr_t RCTaskAttribute = {
    .name = "XuanShuRC",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
    .tz_module = 0,
    .reserved = 0,
};
