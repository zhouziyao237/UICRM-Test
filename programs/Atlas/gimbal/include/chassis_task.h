#pragma once

#include "bsp_thread.h"

/** Read-only chassis state for UI, MiniPC, gimbal feed-forward and diagnostics. */
struct ChassisTelemetry_t {
    bool initialized = false;
    bool online = false;
    bool output_enabled = false;
    float vx = 0.0f;
    float vy = 0.0f;
    float vt = 0.0f;
    float operator_x = 0.0f;
    float operator_y = 0.0f;
    float operator_turn = 0.0f;
    float transformed_x = 0.0f;
    float transformed_y = 0.0f;
    // Final values sent to control::ChassisCanBridgeSender::SetSpeed().
    // The chassis library defines x as right and y as forward.
    float library_x_speed = 0.0f;
    float library_y_speed = 0.0f;
    float frame_offset_deg = 0.0f;
    float relative_yaw = 0.0f;
    float relative_yaw_continuous = 0.0f;
    float chassis_heading = 0.0f;
    float heading_target = 0.0f;
    float heading_error = 0.0f;
    float follow_error = 0.0f;
    bool heading_hold_active = false;
    float battery_voltage = 0.0f;
    float battery_percent = 0.0f;
    bool power_from_referee = false;
    bool power_local_test = false;
    bool power_failsafe = false;
    float power_limit_w = 0.0f;
    float power_buffer_joule = 0.0f;
    float measured_power_w = -1.0f;
    float max_xy_speed = 0.0f;
    float max_turn_speed = 0.0f;
};

void InitChassis();
void ChassisTask(void* arg);
ChassisTelemetry_t ChassisGetTelemetry();
float ChassisGetTurnSpeed();

/**
 * Change the physical chassis-frame calibration while chassis output is off.
 * Returns false when the value is invalid or the chassis is currently enabled.
 */
bool ChassisSetFrameOffsetDeg(float offset_deg);
float ChassisGetFrameOffsetDeg();

extern osThreadId_t ChassisTaskHandle;
inline const osThreadAttr_t ChassisTaskAttribute = {
    .name = "ChassisTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0,
};
