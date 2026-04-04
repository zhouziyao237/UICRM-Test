#include "sys.h"

#include "bsp_os.h"
#include "bsp_print.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "main.h"
#include "rc.h"
#include "referee_task.h"

bsp::CAN* atlas_can1 = nullptr;
bsp::CAN* atlas_can2 = nullptr;
bsp::UART* atlas_referee_uart = nullptr;
bsp::UART* atlas_log_uart = nullptr;
bsp::VirtualUSB* atlas_minipc_usb = nullptr;

bsp::SPI* atlas_imu_spi = nullptr;
bsp::I2C* atlas_magnetometer_i2c = nullptr;
bsp::GPIO* atlas_imu_accel_cs = nullptr;
bsp::GPIO* atlas_imu_gyro_cs = nullptr;
bsp::GPIO* atlas_magnetometer_reset = nullptr;
bsp::GPIT* atlas_imu_accel_interrupt = nullptr;
bsp::GPIT* atlas_imu_gyro_interrupt = nullptr;
bsp::GPIT* atlas_magnetometer_interrupt = nullptr;
bsp::PWM* atlas_imu_heater_pwm = nullptr;

namespace {
void InitCommunicationHardware() {
    atlas_can1 = new bsp::CAN(&hcan1, true);
    atlas_can2 = new bsp::CAN(&hcan2, false);

    atlas_log_uart = new bsp::UART(&huart6);
    atlas_log_uart->SetupRx(300);
    atlas_log_uart->SetBaudrate(921600);
    print_use(atlas_log_uart, true);

#ifdef HAS_REFEREE
    atlas_referee_uart = new bsp::UART(&huart1);
    atlas_referee_uart->SetupRx(300, true);
    atlas_referee_uart->SetupTx(300, true);
#endif

    atlas_minipc_usb = new bsp::VirtualUSB();
    atlas_minipc_usb->SetupRx(300);
    atlas_minipc_usb->SetupTx(300);
}

void InitImuHardware() {
    bsp::spi_init_t spi_config = {&hspi1, bsp::SPI_MODE_DMA};
    atlas_imu_spi = new bsp::SPI(spi_config);

    bsp::i2c_init_t i2c_config = {&hi2c3, bsp::I2C_MODE_IT};
    atlas_magnetometer_i2c = new bsp::I2C(i2c_config);

    atlas_imu_accel_cs = new bsp::GPIO(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);
    atlas_imu_gyro_cs = new bsp::GPIO(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
    atlas_magnetometer_reset = new bsp::GPIO(GPIOG, GPIO_PIN_6);

    atlas_imu_accel_interrupt = new bsp::GPIT(INT1_ACCEL_Pin);
    atlas_imu_gyro_interrupt = new bsp::GPIT(INT1_GYRO_Pin);
    atlas_magnetometer_interrupt = new bsp::GPIT(DRDY_IST8310_Pin);

    atlas_imu_heater_pwm = new bsp::PWM(&htim10, 1, 1000000, 2000, 0);
}
}  // namespace

void InitAtlasHardware() {
    InitRobotMode();
    InitCommunicationHardware();
    InitImuHardware();
}

osThreadId_t SysTaskHandle;

void SysTask(void* arg) {
    UNUSED(arg);
    bool was_killed = true;
    BuzzerPlay(DJI);

    while (true) {
        const RemoteControl_t remote = RCGetSnapshot();
        RefereeSnapshot_t referee{};
#ifdef HAS_REFEREE
        referee = RefereeGetSnapshot();
#endif
        UpdateRobotMode(remote, referee, bsp::GetHighresTickMilliSec());

        const bool killed = RobotIsKilled();
        if (!was_killed && killed) BuzzerPlay(ProtectWarning);
        was_killed = killed;
        osDelay(10);
    }
}
