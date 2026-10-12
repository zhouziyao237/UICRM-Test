#pragma once

#include <cstdint>

#include "bsp_thread.h"

struct VisionTargetSnapshot_t {
    bool online = false;
    bool valid = false;
    float pitch = 0.0f;
    float yaw = 0.0f;
    uint8_t accuracy = 0U;
    uint8_t target_robot_id = 0U;
    uint8_t shoot_cmd = 0U;
    uint32_t source_uptime_ms = 0U;
};

void InitMiniPC();
void MiniPCTask(void* arg);
VisionTargetSnapshot_t VisionGetSnapshot();

extern osThreadId_t MiniPCTaskHandle;
inline const osThreadAttr_t MiniPCTaskAttribute = {
    .name = "XuanShuMiniPC",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 384 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};
