// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by Administrator on 2026/2/27.
//

#include "main.h"
#include "sys.h"
#include "rc.h"

// ----------------------------- sys start data -----------------------------

SysStartData_t SysStartData = {
    .SystemInitFlag = 0,
    .RefereeInitFlag = 0,
    .ImuInitFlag = 0,
    .GimbalInitFlag = 0,
    .ChassisInitFlag = 0,
    .VisionInitFlag = 0,
    .ShootInitFlag = 0,
    .AHRSInitFlag=0,
};

RobotStatus_t RobotStatus = {
    .RemoteOnline = 0,
    .GimbalOnline = 0,
    .ChassisOnline = 0,
    .VisionOnline = 0,
    .RefereeOnline = 0,
    .ShootOnline = 0,
    .RCOnline = 0,
};

GimbalMode_e GimbalMode = GKILLMODE;
ChassisMode_e ChassisMode = CKILLMODE;
ShootMode_e ShootMode = SKILLMODE;

// ----------------------------- sys Hardware Code -----------------------------

// ----------------------------- CAN Code -----------------------------
bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

void InitCAN() {
    can1 = new bsp::CAN(&hcan1, true);
    can2 = new bsp::CAN(&hcan2, false);
}

// ----------------------------- UART Code -----------------------------
bsp::UART* referee_uart = nullptr;

void InitRefereeUart() {
#ifdef RefereePort
    referee_uart = new bsp::UART(&RefereePort);
    referee_uart->SetupRx(300);
    referee_uart->SetupTx(300);
#endif
}

bsp::UART* rc_uart = nullptr;

void InitRCUart() {
#ifdef RCPort
    rc_uart = new bsp::UART(&RCPort);
#endif
#ifdef RCPortRXBUFFERSIZE
    rc_uart->SetupRx(RCPortRXBUFFERSIZE);
#endif
#ifdef RCPortTXBUFFERSIZE
    rc_uart->SetupTx(RCPortTXBUFFERSIZE);
#endif
#ifdef RCPortBaudrate
    rc_uart->SetBaudrate(RCPortBaudrate);
#endif
}

// ----------------------------- USB Code -----------------------------
bsp::VirtualUSB* minipc_usb = nullptr;

void InitMinipcUSB() {
    minipc_usb = new bsp::VirtualUSB();
    minipc_usb->SetupRx(300);
    minipc_usb->SetupTx(300);
}

// ----------------------------- LOG Code -----------------------------

void InitLOGPort() {
#ifdef LOGPort
    print_use_uart(&LOGPort, true, 921600);
#endif
}

// ----------------------------- SPI Code -----------------------------

bsp::spi_init_t SPICFGS[HSPI_NUM] = {HSPI1CFG};
bsp::SPI* SPIS[HSPI_NUM] = {};

void InitSPIPort() {
    for (int i = 0; i < HSPI_NUM; i++) {
        SPIS[i]= new bsp::SPI(SPICFGS[i]);
    }
}

// ----------------------------- I2C Code -----------------------------
#include "bsp_i2c.h"

bsp::i2c_init_t I2CCFGS[I2C_NUM] = {I2C3CFG};
bsp::I2C* I2CS[I2C_NUM] = {};

void InitI2CPort() {
    for (int i = 0; i < I2C_NUM; i++) {
        I2CS[i] = new bsp::I2C(I2CCFGS[i]);
    }
}

// ----------------------------- GPIO Code -----------------------------

GPIO_TypeDef* group[GPIO_NUM] = {CS1_ACCEL_GPIO_Port, CS1_GYRO_GPIO_Port, GPIOG};
uint16_t pin[GPIO_NUM] = {CS1_ACCEL_Pin, CS1_GYRO_Pin, GPIO_PIN_6};
bsp::GPIO* GPIOS[GPIO_NUM] = {};

void InitGPIO() {
    for (int i = 0; i < GPIO_NUM; i++) {
        GPIOS[i] = new bsp::GPIO(group[i], pin[i]);
    }
}

// ----------------------------- INT GPIO Code -----------------------------

uint16_t intpin[INT_GPIO_NUM] = {INT1_ACCEL_Pin, INT1_GYRO_Pin, DRDY_IST8310_Pin};
bsp::GPIT* INTGPIOS[GPIO_NUM] = {};

void InitINTGPIO() {
    for (int i = 0; i < INT_GPIO_NUM; i++) {
        INTGPIOS[i] = new bsp::GPIT(intpin[i]);
    }
}

// ----------------------------- PWM GPIO Code -----------------------------

PWMCFG_t PWMCFG[PWM_GPIO_NUM] = {heater_pwm_cfg};
bsp::PWM* PWMS[PWM_GPIO_NUM] = {};

void InitPWMGPIO() {
    for (int i = 0; i < PWM_GPIO_NUM; i++) {
         PWMS[i] = new bsp::PWM(
                 PWMCFG[i].htim,
                 PWMCFG[i].channel,
                 PWMCFG[i].clock_freq,
                 PWMCFG[i].output_freq,
                 PWMCFG[i].pulse_width);
    }
}

// ---------------------------- sys Init Code ----------------------------

void InitRMSys() {
    InitLOGPort();
    InitCAN();
    InitRCUart();
    InitRefereeUart();
    InitMinipcUSB();

    InitGPIO();
    InitINTGPIO();
    InitPWMGPIO();
    InitSPIPort();
    InitI2CPort();

}

// ---------------------------- sys Task Code ----------------------------

osThreadId_t SysTaskHandle;

RobotMode_e RobotMode = RKILLMODE;

bool rc_offline = false;

// ---------------------------- Sys Main Code ----------------------------

void SysInit() {

}

void SysTask(void* arg) {
    UNUSED(arg);

    while (!RCIsOnline()) {
        RobotMode = RKILLMODE;
        ChassisMode = CKILLMODE;
        GimbalMode = GKILLMODE;
        ShootMode = SKILLMODE;
        osDelay(10);
    }

    if (rcData[MODE_SWITCH_CHANNEL] != MODE_KILL) {
        while (true) {

            if (rcData[MODE_SWITCH_CHANNEL] == MODE_KILL) {
                RobotMode = RMANUALMODE;
                GimbalMode = GMANUALMODE;
                osDelay(100);
                break;
            }

            RobotMode = RKILLMODE;
            ChassisMode = CKILLMODE;
            GimbalMode = GKILLMODE;
            ShootMode = SKILLMODE;
            Buzzer_Sing(ProtectWarning);
            osDelay(100);
        }
    }

    osDelay(500);
    Buzzer_Sing(DJI);

    while (true) {
        rc_offline = (!RCIsOnline() || rcData[MODE_SWITCH_CHANNEL] == MODE_KILL);


        osDelay(1);
    }
}