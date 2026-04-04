#pragma once

#include <cstdint>

/**
 * Read-only IMU/AHRS output exposed to the rest of Atlas.
 *
 * Hardware driver pointers and AHRS ownership are private to imu_task.cpp.
 */
struct ImuData_t {
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    float gyro[3] = {};
    float accel[3] = {};
    float temperature = 0.0f;
    bool online = false;
    bool calibrated = false;
};

void InitIMU();
ImuData_t ImuGetData();
bool ImuIsOnline();
void ImuStartCalibration();
bool ImuCalibrationDone();
