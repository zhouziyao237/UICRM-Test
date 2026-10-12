#pragma once

#include <cstdint>

#include "bsp_thread.h"

/**
 * Optional future chassis-bus current reader.
 * Return true and write signed current in ampere when a fresh sample exists.
 * Leave unregistered for the current MODEL + REFEREE observer mode.
 */
using PowerCurrentReader_t = bool (*)(float* current_amp);

void InitPower();
void PowerTask(void* arg);

/**
 * Future sensor hook. Installing a real lower-board chassis-current sensor
 * only changes this input adapter; Kalman/LQR/Hard-Guard code stays unchanged.
 */
void PowerControlRegisterCurrentReader(PowerCurrentReader_t reader);
bool PowerControlIsBridgeStale();
uint32_t PowerControlGetBridgeAgeMs();

extern osThreadId_t PowerTaskHandle;
inline const osThreadAttr_t PowerTaskAttribute = {
    .name = "PowerTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 256 * 4,
    // MotorUpdateTask remains the final higher-priority CAN hard-guard owner.
    .priority = (osPriority_t)osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};
