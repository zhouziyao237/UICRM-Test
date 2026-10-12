#pragma once

#include <cstdint>

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
