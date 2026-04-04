#pragma once

#include <cstdint>

#include "bsp_thread.h"

/** Shooter action state. This is owned only by ShootTask. */
enum ShootMode_e : int8_t {
    SHOOT_MODE_UNLOAD = -1,
    SHOOT_MODE_IDLE = 1,
    SHOOT_MODE_SINGLE = 2,
    SHOOT_MODE_DOUBLE = 3,
    SHOOT_MODE_RAPID = 4,
};

struct ShooterTelemetry_t {
    bool initialized = false;
    bool online = false;
    bool output_enabled = false;
    bool feeder_online = false;
    bool friction_enabled = false;
    bool jammed = false;
    ShootMode_e mode = SHOOT_MODE_IDLE;
};

void InitShoot();
void ShootTask(void* arg);
uint8_t ShootIsOnline();
ShooterTelemetry_t ShooterGetTelemetry();

extern osThreadId_t ShootTaskHandle;
inline const osThreadAttr_t ShootTaskAttribute = {
    .name = "ShootTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};
