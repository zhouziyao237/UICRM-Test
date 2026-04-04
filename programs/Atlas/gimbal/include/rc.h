#pragma once

#include <cstdint>

#include "bsp_thread.h"

enum RemoteFireCommand_e : uint8_t {
    REMOTE_FIRE_NONE = 0,
    REMOTE_FIRE_SINGLE,
    REMOTE_FIRE_DOUBLE,
    REMOTE_FIRE_RAPID,
};

enum RemoteUiCommand_e : uint8_t {
    REMOTE_UI_NONE = 0,
    REMOTE_UI_CLEAR_GRAPHICS,
    REMOTE_UI_CLEAR_DIAGNOSTICS,
};

/**
 * Normalized DT7/DR16 values consumed directly by the control tasks.
 * Channel numbers, signs and DBUS raw ranges stay inside rc.cpp.
 */
struct RemoteControl_t {
    bool online = false;
    uint32_t generation = 0;
    bool physical_kill = true;

    bool cycle_mode = false;
    bool manual_mode = false;
    bool spin_mode = false;
    bool follow_mode = false;

    float chassis_forward = 0.0f;
    float chassis_lateral = 0.0f;
    float chassis_turn = 0.0f;
    bool keyboard_motion = false;
    float gimbal_pitch_delta = 0.0f;
    float gimbal_yaw_delta = 0.0f;

    bool friction_toggle = false;
    bool unload = false;

    RemoteFireCommand_e fire_command = REMOTE_FIRE_NONE;
    uint32_t fire_sequence = 0;
    RemoteUiCommand_e ui_command = REMOTE_UI_NONE;
    uint32_t ui_sequence = 0;
};

void InitRC();
void RCTask(void* arg);
RemoteControl_t RCGetSnapshot();
const char* RCSourceName();

extern osThreadId_t RCTaskHandle;
inline const osThreadAttr_t RCTaskAttribute = {
    .name = "RCTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 512 * 2,
    .priority = (osPriority_t)osPriorityHigh,
    .tz_module = 0,
    .reserved = 0,
};
