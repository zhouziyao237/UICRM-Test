#pragma once

#include "bsp_thread.h"

/** Read-only gimbal state for chassis, UI, MiniPC and diagnostics. */
struct GimbalTelemetry_t {
    bool initialized = false;
    bool motors_online = false;
    bool pitch_online = false;
    bool yaw_online = false;
    bool output_enabled = false;
    float pitch_relative = 0.0f;
    float yaw_relative = 0.0f;
    float pitch_target = 0.0f;
    float yaw_target = 0.0f;
    float pitch_motor_theta = 0.0f;
    float yaw_motor_theta = 0.0f;
    float pitch_motor_omega = 0.0f;
    int16_t pitch_current = 0;
};

void InitGimbal();
void GimbalTask(void* arg);
uint8_t GimbalIsOnline();
GimbalTelemetry_t GimbalGetTelemetry();
float GimbalGetRelativeYaw();

extern osThreadId_t GimbalTaskHandle;
inline const osThreadAttr_t GimbalTaskAttribute = {
    .name = "GimbalTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};
