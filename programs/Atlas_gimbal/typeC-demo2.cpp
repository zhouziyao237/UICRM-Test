/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "bsp_print.h"
#include "cmsis_os.h"
#include "gimbal.h"
#include "main.h"
#include "vtm.h"

#include "spi.h"
#include "i2c.h"
#include "AHRS.h"
#include "BMI088.h"
#include "IST8310.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "heater.h"


static bsp::CAN* can1 = nullptr;
static bsp::UART* uart = nullptr;
static remote::VTM* vtm = nullptr;

static control::AHRS* ahrs = nullptr;
static driver::Heater* heater = nullptr;
static bsp::PWM* heater_pwm = nullptr;

static bsp::SPI* spi1 = nullptr;
static bsp::SPIMaster* spi1_master = nullptr;
static imu::BMI088* bmi088 = nullptr;
static bsp::GPIO* bmi088_accel_cs = nullptr;
static bsp::GPIO* bmi088_gyro_cs = nullptr;
static bsp::GPIT* bmi088_accel_int = nullptr;
static bsp::GPIT* bmi088_gyro_int = nullptr;

static bsp::I2C* i2c3 = nullptr;
static bsp::GPIO* ist8310_rst = nullptr;
static bsp::GPIT* ist8310_int = nullptr;
static imu::IST8310* ist8310 = nullptr;

void BMI088ReceiveDone() {
    ahrs->Update(bmi088->gyro_[0], bmi088->gyro_[1], bmi088->gyro_[2], bmi088->accel_[0],
                 bmi088->accel_[1], bmi088->accel_[2]);
    heater->Update(bmi088->temperature_);
}

static driver::MotorCANBase* pitch_motor = nullptr;
static driver::MotorCANBase* yaw_motor = nullptr;
static control::Gimbal* gimbal = nullptr;
static control::gimbal_data_t* gimbal_param = nullptr;

enum mode {
    MANUALMODE = 0,
    IMUMODE = 1
};

uint8_t manual_mode = MANUALMODE;

// 配置你的云台（±60度，1:6减速比）
control::gimbal_data_t gimbal_config = {
    .pitch_offset_ = 1.9225f,        // 云台机械零点
    .yaw_offset_ = 0.0f,           // 云台机械零点
    .pitch_max_ = DEG_TO_RAD(40),  // ±40度范围
    .yaw_max_ = 2 * PI,            // ±2PI度范围
    .yaw_circle_ = false,          // 不能360度旋转
    .pitch_inverted = true,        // 不反转
    .yaw_inverted = false,         // 不反转
    .pitch_reduction = 6.0f,   // 1:6减速比
    .yaw_reduction = 1.0f          // 1:6减速比
};

const osThreadAttr_t gimbalTaskAttribute = {.name = "gimbalTask",
                                            .attr_bits = osThreadDetached,
                                            .cb_mem = nullptr,
                                            .cb_size = 0,
                                            .stack_mem = nullptr,
                                            .stack_size = 256 * 4,
                                            .priority = (osPriority_t)osPriorityNormal,
                                            .tz_module = 0,
                                            .reserved = 0};
osThreadId_t gimbalTaskHandle;

void gimbalTask(void* arg) {
    UNUSED(arg);

    print("Wait for beginning signal...\r\n");
    while (true) {
        if (vtm->keyboard.bit.V || vtm->ch5 == 1) {
            break;
        }
        osDelay(100);
    }

    int i = 0;
    for (i = 0; i < 5000; i++) {
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        osDelay(1);
    }

    ahrs->Cailbrate();
    i = 0;
    while (!ahrs->IsCailbrated()) {
        gimbal->TargetAbs(0, 0);
        gimbal->Update();
        osDelay(1);
        ++i;
    }

    print("Start Calibration.\r\n");
    ahrs->Cailbrate();

    print("Gimbal Begin!\r\n");

    float pitch_ratio, yaw_ratio;
    float pitch_curr, yaw_curr;
    float pitch_diff, yaw_diff;
    float pitch_target = 0, yaw_target = 0;



    while (true) {
        if (vtm->keyboard.bit.B || vtm->gear_switch == 0) {
            while (true) {
                if (vtm->keyboard.bit.V || vtm->ch5 == 1) {
                    break;
                }
                osDelay(10);
            }
        }

        if (vtm->custom_right == 1) {
            manual_mode = MANUALMODE;
        }

        if (vtm->custom_left == 1) {
            manual_mode = IMUMODE;
        }


        pitch_ratio = vtm->ch2 / 1800.0 / 2.0;
        yaw_ratio = -vtm->ch3 / 1800.0 / 2.0;
        pitch_target = clip<float>(pitch_ratio, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        yaw_target = clip<float>(yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);

        switch (manual_mode) {
            case MANUALMODE: {
                // 手动控制模式
                gimbal->TargetAbs(pitch_target, yaw_target);
                gimbal->Update();
            }break;

            case IMUMODE: {
                // IMU跟随模式
                pitch_diff = wrap<float>(pitch_target, -PI, PI);
                yaw_diff = wrap<float>(yaw_target, -PI, PI);
                pitch_curr = ahrs->INS_angle[1];
                yaw_curr = ahrs->INS_angle[0];
                gimbal->TargetRel(pitch_diff, yaw_diff);
                gimbal->UpdateIMU(pitch_curr, yaw_curr);
            }break;
            default:
                break;

        }

        osDelay(1);
    }
}

void RM_RTOS_Init(void) {
    print_use_uart(&huart1, true, 921600);

    can1 = new bsp::CAN(&hcan1, true);

    uart = new bsp::UART(&huart6);
    uart->SetBaudrate(912600);
    uart->SetupRx(BUFFER_SIZE);
    vtm = new remote::VTM(uart);

    bsp::spi_init_t spiInit = {
        .hspi = &hspi1,
        .mode = bsp::SPI_MODE_DMA,
    };
    spi1 = new bsp::SPI(spiInit);
    bsp::spi_master_init_t spiMasterInit = {
        .spi = spi1,
    };
    spi1_master = new bsp::SPIMaster(spiMasterInit);
    bmi088_accel_cs = new bsp::GPIO(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);
    bmi088_gyro_cs = new bsp::GPIO(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
    bmi088_accel_int = new bsp::GPIT(INT1_ACCEL_Pin);
    bmi088_gyro_int = new bsp::GPIT(INT1_GYRO_Pin);
    imu::BMI088_init_t BMI088_init = {
        .spi_master = spi1_master,
        .CS_ACCEL = bmi088_accel_cs,
        .CS_GYRO = bmi088_gyro_cs,
        .INT_ACCEL = bmi088_accel_int,
        .INT_GYRO = bmi088_gyro_int,
        .is_DMA = true,
    };

    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    ist8310_rst = new bsp::GPIO(GPIOG, GPIO_PIN_6);
    ist8310_int = new bsp::GPIT(DRDY_IST8310_Pin);
    bsp::i2c_init_t i2c3_init = {.hi2c = &hi2c3, .mode = bsp::I2C_MODE_IT};
    i2c3 = new bsp::I2C(i2c3_init);
    imu::IST8310_init_t ist8310Init = {.i2c = i2c3, .drdy = ist8310_int, .rst = ist8310_rst};

    ist8310 = new imu::IST8310(ist8310Init);
    bmi088 = new imu::BMI088(BMI088_init);

    ahrs = new control::AHRS(false);
    heater_pwm = new bsp::PWM(&htim10, 1, 1000000, 2000, 0);
    driver::heater_init_t heater_init = {
        .pwm = heater_pwm,
        .target_temp = 45.0f,
    };
    heater = new driver::Heater(heater_init);
    bmi088->RegisterCallback(BMI088ReceiveDone);


    pitch_motor = new driver::Motor3508(can1, 0x204);
    yaw_motor = new driver::Motor6020(can1, 0x205);

    pitch_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t theta_pid_init = {
        .kp = 120,
        .ki = 0,
        .kd = 58,
        .max_out = 15 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    pitch_motor->ReInitPID(theta_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 120,
        .ki = 0,
        .kd = 2500,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,                          // 死区
        .A = 1.5 * PI,                            // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,    // 输出滤波系数
        .derivative_filtering_coefficient = 0,  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };
    pitch_motor->ReInitPID(omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    pitch_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                         driver::MotorCANBase::ANGLE_LOOP_CONTROL | driver::MotorCANBase::ABSOLUTE);

    yaw_motor->SetTransmissionRatio(1);
    control::ConstrainedPID::PID_Init_t yaw_theta_pid_init = {
        .kp = 20,
        .ki = 0,
        .kd = 0,
        .max_out = 6 * PI,
        .max_iout = 0,
        .deadband = 0,                                 // 死区
        .A = 0,                                        // 变速积分所能达到的最大值为A+B
        .B = 0,                                        // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,           // 输出滤波系数
        .derivative_filtering_coefficient = 0,         // 微分滤波系数
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };
    yaw_motor->ReInitPID(yaw_theta_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    control::ConstrainedPID::PID_Init_t yaw_omega_pid_init = {
        .kp = 2500,
        .ki = 3,
        .kd = 0,
        .max_out = 30000,
        .max_iout = 10000,
        .deadband = 0,                                          // 死区
        .A = 1.5 * PI,                                          // 变速积分所能达到的最大值为A+B
        .B = 1 * PI,                                            // 启动变速积分的死区
        .output_filtering_coefficient = 0.1,                    // 输出滤波系数
        .derivative_filtering_coefficient = 0,                  // 微分滤波系数
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };
    yaw_motor->ReInitPID(yaw_omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    yaw_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                       driver::MotorCANBase::ANGLE_LOOP_CONTROL | driver::MotorCANBase::ABSOLUTE);

    control::gimbal_t gimbal_data;
    gimbal_data.data = gimbal_config;
    gimbal_data.pitch_motor = pitch_motor;
    gimbal_data.yaw_motor = yaw_motor;
    gimbal = new control::Gimbal(gimbal_data);
    gimbal_param = gimbal->GetData();
}

void RM_RTOS_Threads_Init(void) {
    gimbalTaskHandle = osThreadNew(gimbalTask, nullptr, &gimbalTaskAttribute);
}

void KillAll() {
    RM_EXPECT_TRUE(false, "Operation killed\r\n");
    while (true) {
        if (vtm->keyboard.bit.V || vtm->ch5 == 1) {
            pitch_motor->Enable();
            yaw_motor->Enable();
            break;
        }
        pitch_motor->Disable();
        yaw_motor->Disable();
        osDelay(10);
    }
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);

    while (true) {
        if (vtm->keyboard.bit.B || vtm->gear_switch == 0)
            KillAll();

        set_cursor(0, 0);
        clear_screen();


        print("Gimbal Model: %s\r\n", manual_mode ? "IMU" : "MANUAL");
        print("# %.2f s, IMU %s\r\n", HAL_GetTick() / 1000.0,
              ahrs->IsCailbrated() ? "\033[1;42mReady\033[0m" : "\033[1;41mNot Ready\033[0m");
        print("Temp: %.2f\r\n", bmi088->temperature_);
        print("Euler Angles: %.2f, %.2f, %.2f\r\n", ahrs->INS_angle[0] / PI * 180,
              ahrs->INS_angle[1] / PI * 180, ahrs->INS_angle[2] / PI * 180);

        print("\r\n");

        bool is_connected = vtm->IsOnline();
        print("VTM Status: %s | Updates: %lu\r\n", is_connected ? "CONNECTED" : "DISCONNECTED",
              vtm->timestamp);

        print(
            "CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d \r\n"
            "CH4: %-4d CH5: %-4d GSWL: %d PBUT: %d \r\n"
            "Custom: L=%d R=%d\r\n",  // 添加自定义按键显示
            vtm->ch0, vtm->ch1, vtm->ch2, vtm->ch3, vtm->ch4, vtm->ch5, vtm->gear_switch,
            vtm->pause_button, vtm->custom_left, vtm->custom_right);  // 添加自定义按键值

        print("W: %d A: %d S: %d D: %d\r\n", vtm->keyboard.bit.W, vtm->keyboard.bit.A,
              vtm->keyboard.bit.S, vtm->keyboard.bit.D);
        print("Q: %d E: %d R: %d F: %d G: %d\r\n", vtm->keyboard.bit.Q, vtm->keyboard.bit.E,
              vtm->keyboard.bit.R, vtm->keyboard.bit.F, vtm->keyboard.bit.G);
        print("Z: %d X: %d C: %d V: %d B: %d\r\n", vtm->keyboard.bit.Z, vtm->keyboard.bit.X,
              vtm->keyboard.bit.C, vtm->keyboard.bit.V, vtm->keyboard.bit.B);
        print("SHIFT: %d CTRL: %d\r\n", vtm->keyboard.bit.SHIFT, vtm->keyboard.bit.CTRL);
        print("Mouse: x: %d y: %d z: %d l: %d r: %d\r\n", vtm->mouse.x, vtm->mouse.y, vtm->mouse.z,
              vtm->mouse.l, vtm->mouse.r);

        char buffer[128];
        gimbal->GetDebugString(buffer, sizeof(buffer));
        print("%s\n", buffer);

        osDelay(100);
    }
}
