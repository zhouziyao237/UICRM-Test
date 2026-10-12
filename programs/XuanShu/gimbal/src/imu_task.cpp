#include "imu_task.h"

#include "AHRS.h"
#include "BMI088.h"
#include "bsp_gpio.h"
#include "bsp_error_handler.h"
#include "bsp_os.h"
#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "gimbal_data.h"
#include "heater.h"
#include "main.h"
#include "cmsis_os.h"
#include "task.h"

namespace cfg = xuanshu::gimbal_config;

namespace {
control::AHRS* ahrs = nullptr;
driver::Heater* heater = nullptr;
bsp::SPI* imu_spi = nullptr;
bsp::SPIMaster* spi_master = nullptr;
bsp::GPIO* accel_cs = nullptr;
bsp::GPIO* gyro_cs = nullptr;
bsp::GPIT* accel_interrupt = nullptr;
bsp::GPIT* gyro_interrupt = nullptr;
bsp::PWM* heater_pwm = nullptr;
imu::BMI088* bmi088 = nullptr;

ImuData_t frames[2]{};
volatile uint8_t active_frame = 0U;
volatile uint32_t last_update_ms = 0U;

void PublishFromSensor() {
    if (ahrs == nullptr || heater == nullptr || bmi088 == nullptr) return;

    ahrs->Update(bmi088->gyro_[0], bmi088->gyro_[1], bmi088->gyro_[2],
                 bmi088->accel_[0], bmi088->accel_[1], bmi088->accel_[2]);
    heater->Update(bmi088->temperature_);

    ImuData_t next;
    next.yaw = cfg::kImuYawSign * ahrs->INS_angle[cfg::kImuYawAxis];
    next.pitch = cfg::kImuPitchSign * ahrs->INS_angle[cfg::kImuPitchAxis];
    next.roll = cfg::kImuRollSign * ahrs->INS_angle[cfg::kImuRollAxis];
    for (uint8_t i = 0U; i < 3U; ++i) {
        next.gyro[i] = bmi088->gyro_[i];
        next.accel[i] = bmi088->accel_[i];
    }
    next.temperature = bmi088->temperature_;
    next.calibrated = ahrs->IsCailbrated();
    next.online = true;

    taskENTER_CRITICAL();
    const uint8_t next_index = static_cast<uint8_t>(active_frame ^ 1U);
    frames[next_index] = next;
    active_frame = next_index;
    last_update_ms = bsp::GetHighresTickMilliSec();
    taskEXIT_CRITICAL();
}
}  // namespace

void InitIMU() {
    HAL_Delay(300);

    bsp::spi_init_t spi_config = {&hspi1, bsp::SPI_MODE_DMA};
    imu_spi = new bsp::SPI(spi_config);
    bsp::spi_master_init_t master_config = {.spi = imu_spi};
    spi_master = new bsp::SPIMaster(master_config);

    accel_cs = new bsp::GPIO(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);
    gyro_cs = new bsp::GPIO(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
    accel_interrupt = new bsp::GPIT(INT1_ACCEL_Pin);
    gyro_interrupt = new bsp::GPIT(INT1_GYRO_Pin);

    imu::BMI088_init_t bmi088_init = {
        .spi_master = spi_master,
        .CS_ACCEL = accel_cs,
        .CS_GYRO = gyro_cs,
        .INT_ACCEL = accel_interrupt,
        .INT_GYRO = gyro_interrupt,
        .is_DMA = true,
    };
    bmi088 = new imu::BMI088(bmi088_init);
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

    ahrs = new control::AHRS(false);
    heater_pwm = new bsp::PWM(&htim10, 1, 1000000, 2000, 0);
    driver::heater_init_t heater_init = {
        .pwm = heater_pwm,
        .target_temp = cfg::kImuHeaterTargetC,
    };
    heater = new driver::Heater(heater_init);

    RM_ASSERT_TRUE(bmi088 != nullptr && ahrs != nullptr && heater != nullptr,
                   "XuanShu IMU init failed");
    bmi088->RegisterCallback(PublishFromSensor);
    ImuStartCalibration();
}

bool ImuIsOnline() {
    uint32_t last = 0U;
    taskENTER_CRITICAL();
    last = last_update_ms;
    taskEXIT_CRITICAL();
    return last != 0U &&
           bsp::GetHighresTickMilliSec() - last <= cfg::kImuOfflineTimeoutMs;
}

ImuData_t ImuGetData() {
    taskENTER_CRITICAL();
    ImuData_t data = frames[active_frame];
    taskEXIT_CRITICAL();
    data.online = ImuIsOnline();
    data.calibrated = ImuCalibrationDone();
    return data;
}

void ImuStartCalibration() {
    if (ahrs != nullptr) ahrs->Cailbrate();
}

bool ImuCalibrationDone() {
    return ahrs != nullptr && ahrs->IsCailbrated();
}
