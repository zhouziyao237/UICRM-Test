#pragma once

#include <cstdint>

#include "bsp_thread.h"

/**
 * Atlas upper-board power transport state.
 *
 * This task does NOT run Kalman/LQR and does NOT scale motor current.  It only
 * forwards the newest referee power state to the lower chassis board with low
 * scheduling latency. Voltage/current acquisition belongs to the lower chassis board.
 */
struct PowerForwardTelemetry_t {
    bool initialized = false;
    bool referee_online = false;
    bool power_from_referee = false;
    bool power_local_test = false;
    bool power_failsafe = true;
    float power_limit_w = 0.0f;
    float power_buffer_joule = 0.0f;
    float measured_power_w = -1.0f;
    uint32_t referee_power_sequence = 0U;
    uint32_t forwarded_power_sequence = 0U;
    uint32_t last_limit_forward_ms = 0U;
    uint32_t last_power_forward_ms = 0U;
};

void InitPower();
void PowerTask(void* arg);
PowerForwardTelemetry_t PowerForwardGetTelemetry();

extern osThreadId_t PowerTaskHandle;
inline const osThreadAttr_t PowerTaskAttribute = {
    .name = "PowerTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 256 * 4,
    // Very short non-blocking forwarding task.  Keep it above normal control
    // tasks, but below SysTask's highest safety/state priority.
    .priority = (osPriority_t)osPriorityHigh4,
    .tz_module = 0,
    .reserved = 0,
};
