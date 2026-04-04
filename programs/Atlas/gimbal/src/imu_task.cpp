#include "imu_task.h"

#include "AHRS.h"
#include "BMI088.h"
#include "IST8310.h"
#include "bsp_os.h"
#include "heater.h"
#include "sys.h"

namespace {
constexpr uint32_t kImuOfflineTimeoutMs = 100;

// Axis mapping from the current Atlas IMU installation to gimbal world axes.
constexpr uint8_t kYawAxis = 0;
constexpr uint8_t kPitchAxis = 1;
constexpr uint8_t kRollAxis = 2;
constexpr float kYawSign = 1.0f;
// Real-car verification: Atlas logical pitch is positive when the muzzle moves
// upward, while the installed BMI088/AHRS pitch decreases in that direction.
// Convert the IMU measurement once here so IMU pitch and gimbal logical pitch
// share the same sign. Without this inversion FOLLOW/SPIN forms positive
// feedback and drives pitch into the upper mechanical limit.
constexpr float kPitchSign = -1.0f;
constexpr float kRollSign = 1.0f;

control::AHRS* g_ahrs = nullptr;
driver::Heater* g_heater = nullptr;
bsp::SPIMaster* g_spi_master = nullptr;
imu::BMI088* g_bmi088 = nullptr;
imu::IST8310* g_ist8310 = nullptr;

volatile float g_pitch = 0.0f;
volatile float g_roll = 0.0f;
volatile float g_yaw = 0.0f;
volatile float g_gyro[3] = {};
volatile float g_accel[3] = {};
volatile float g_temperature = 0.0f;
volatile uint32_t g_last_update_ms = 0;

void BMI088ReceiveDone() {
    if (g_ahrs == nullptr || g_heater == nullptr || g_bmi088 == nullptr) {
        return;
    }

    g_ahrs->Update(g_bmi088->gyro_[0], g_bmi088->gyro_[1], g_bmi088->gyro_[2],
                   g_bmi088->accel_[0], g_bmi088->accel_[1], g_bmi088->accel_[2]);
    g_heater->Update(g_bmi088->temperature_);

    for (uint8_t i = 0; i < 3; ++i) {
        g_gyro[i] = g_bmi088->gyro_[i];
        g_accel[i] = g_bmi088->accel_[i];
    }
    g_temperature = g_bmi088->temperature_;
    g_yaw = kYawSign * g_ahrs->INS_angle[kYawAxis];
    g_pitch = kPitchSign * g_ahrs->INS_angle[kPitchAxis];
    g_roll = kRollSign * g_ahrs->INS_angle[kRollAxis];
    g_last_update_ms = bsp::GetHighresTickMilliSec();
}
}  // namespace

void InitIMU() {
    HAL_Delay(500);

    bsp::spi_master_init_t spi_master_init = {
        .spi = atlas_imu_spi,
    };
    g_spi_master = new bsp::SPIMaster(spi_master_init);

    imu::BMI088_init_t bmi088_init = {
        .spi_master = g_spi_master,
        .CS_ACCEL = atlas_imu_accel_cs,
        .CS_GYRO = atlas_imu_gyro_cs,
        .INT_ACCEL = atlas_imu_accel_interrupt,
        .INT_GYRO = atlas_imu_gyro_interrupt,
        .is_DMA = true,
    };
    g_bmi088 = new imu::BMI088(bmi088_init);
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

    imu::IST8310_init_t ist8310_init = {
        .i2c = atlas_magnetometer_i2c,
        .drdy = atlas_magnetometer_interrupt,
        .rst = atlas_magnetometer_reset,
    };
    g_ist8310 = new imu::IST8310(ist8310_init);

    g_ahrs = new control::AHRS(false);

    driver::heater_init_t heater_init = {
        .pwm = atlas_imu_heater_pwm,
        .target_temp = 45.0f,
    };
    g_heater = new driver::Heater(heater_init);

    RM_ASSERT_TRUE(g_bmi088 != nullptr && g_ahrs != nullptr && g_heater != nullptr,
                   "IMU init failed");

    g_bmi088->RegisterCallback(BMI088ReceiveDone);
    osDelay(500);
}

bool ImuIsOnline() {
    const uint32_t last_update = g_last_update_ms;
    const bool online = last_update != 0 &&
                        bsp::GetHighresTickMilliSec() - last_update <= kImuOfflineTimeoutMs;
    return online;
}

ImuData_t ImuGetData() {
    ImuData_t data;
    data.pitch = g_pitch;
    data.roll = g_roll;
    data.yaw = g_yaw;
    for (uint8_t i = 0; i < 3; ++i) {
        data.gyro[i] = g_gyro[i];
        data.accel[i] = g_accel[i];
    }
    data.temperature = g_temperature;
    data.online = ImuIsOnline();
    data.calibrated = ImuCalibrationDone();
    return data;
}

void ImuStartCalibration() {
    if (g_ahrs != nullptr) {
        g_ahrs->Cailbrate();
    }
}

bool ImuCalibrationDone() {
    return g_ahrs != nullptr && g_ahrs->IsCailbrated();
}
