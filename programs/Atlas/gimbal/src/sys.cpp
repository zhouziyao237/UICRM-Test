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

#include "sys.h"

#include "main.h"
#include "rc.h"

// ----------------------------- sys start data -----------------------------

volatile SysStartData_t SysStartData = {
    .SystemInitFlag = 0,
    .RefereeInitFlag = 0,
    .ImuInitFlag = 0,
    .GimbalInitFlag = 0,
    .ChassisInitFlag = 0,
    .VisionInitFlag = 0,
    .ShootInitFlag = 0,
    .AHRSInitFlag = 0,
};

volatile RobotStatus_t RobotStatus = {
    .RemoteOnline = 0,
    .GimbalOnline = 0,
    .ChassisOnline = 0,
    .VisionOnline = 0,
    .RefereeOnline = 0,
    .ShootOnline = 0,
    .RCOnline = 0,
};

volatile GimbalMode_e GimbalMode = GKILLMODE;
volatile ChassisMode_e ChassisMode = CKILLMODE;
volatile ShootMode_e ShootMode = SKILLMODE;

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
        SPIS[i] = new bsp::SPI(SPICFGS[i]);
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
        PWMS[i] = new bsp::PWM(PWMCFG[i].htim, PWMCFG[i].channel, PWMCFG[i].clock_freq,
                               PWMCFG[i].output_freq, PWMCFG[i].pulse_width);
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

volatile RobotMode_e RobotMode = RMANUALMODE;
volatile RobotMode_e LastRobotMode = RMANUALMODE;

BoolEdgeDetector* RoboModeSwitch_CH = nullptr;
BoolEdgeDetector* RoboModeSwitch_KEY[SwitchKEYNUM] = {nullptr};

bool rc_offline = false;
bool is_killed = false;
bool is_robot_dead = false;
bool is_shoot_available = false;

const int8_t remote_mode_max = 1;
const int8_t remote_mode_min = 4;

BoolEdgeDetector* game_start_edge = nullptr;

// ---------------------------- Sys Main Code ----------------------------

void RoboSysInit() {
    RoboModeSwitch_CH = new BoolEdgeDetector(false);
    for (int i = 0; i < SwitchKEYNUM; i++) {
        RoboModeSwitch_KEY[i] = new BoolEdgeDetector(false);
    }
    game_start_edge = new BoolEdgeDetector(false);
}

void SwitchRobotMode() {
    switch (RobotMode) {
        case RMANUALMODE: {
            ChassisMode = CMANUALMODE;
            GimbalMode = GMANUALMODE;
            ShootMode = SHOOTILDE;
        } break;
        case RAUTOMODE: {
            ChassisMode = CMANUALMODE;
            GimbalMode = GAUTOAIMODE;
            ShootMode = SHOOTILDE;
        } break;
        case RSPINMODE: {
            ChassisMode = CSPINMODE;
            GimbalMode = GSPINMODE;
            ShootMode = SHOOTILDE;
        } break;
        case RFOLLOWMODE: {
            ChassisMode = CFOLLOWMODE;
            GimbalMode = GFOLLOWMODE;
            ShootMode = SHOOTILDE;
        } break;
        default:
            break;
    }
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

    osDelay(500);

#ifdef HAS_REFEREE
    if (RefereeIsOnline()) {
        while (!referee->game_robot_status.mains_power_gimbal_output) {
            RobotMode = RKILLMODE;
            ChassisMode = CKILLMODE;
            GimbalMode = GKILLMODE;
            ShootMode = SKILLMODE;
            osDelay(10);
        }
    }
#endif

    if (rcData[MODE_SWITCH_CHANNEL] != MODE_KILL) {
        while (true) {
            if (rcData[MODE_SWITCH_CHANNEL] == MODE_KILL) {
                osDelay(100);
                break;
            }

            Buzzer_Sing(ProtectWarning);
            osDelay(100);
        }
    }

    osDelay(500);
    Buzzer_Sing(DJI);

    while (true) {
        rc_offline = !RCIsOnline();

#ifdef HAS_REFEREE
        game_start_edge->input(referee->game_status.game_progress == 4);
        if (game_start_edge && RefereeIsOnline()) {
             is_robot_dead =
            referee->game_robot_status.remain_HP == 0 || rcData[MODE_SWITCH_CHANNEL] == MODE_KILL;
        }else {
            is_robot_dead = rcData[MODE_SWITCH_CHANNEL] == MODE_KILL;
        }
       #else
        is_robot_dead = rcData[MODE_SWITCH_CHANNEL] == MODE_KILL;
#endif

        if (rc_offline || is_robot_dead) {
            if (!is_killed) {
                LastRobotMode = RobotMode;
                RobotMode = RKILLMODE;
                ChassisMode = CKILLMODE;
                GimbalMode = GKILLMODE;
                ShootMode = SKILLMODE;
                is_killed = true;
            }
        } else {
            if (is_killed) {
                RobotMode = LastRobotMode;
                SwitchRobotMode();

                is_killed = false;
            }
        }

        if (RobotMode == RKILLMODE || is_killed) {
            osDelay(10);
            continue;
        }

        RoboModeSwitch_KEY[SwitchKEY]->input(rcData[ROBOMODE_SWITCH_CH]);
        RoboModeSwitch_CH->input(rcData[ROBOMODE_SWITCH_KEY]);

        if (RoboModeSwitch_KEY[SwitchKEY]->posEdge() || RoboModeSwitch_CH->posEdge()) {
            RobotMode_e next_mode = (RobotMode_e)(RobotMode + 1);
            if ((int8_t)next_mode > (int8_t)remote_mode_max) {
                next_mode = (RobotMode_e)remote_mode_min;
            }
            RobotMode = next_mode;
            SwitchRobotMode();
        }

        RoboModeSwitch_KEY[FOLLOWMODEKEY]->input(rcData[KEYBOARD_C_CHANNEL]);
        RoboModeSwitch_KEY[RSPINMODEKEY]->input(rcData[KEYBOARD_V_CHANNEL]);
        RoboModeSwitch_KEY[MANUALMODEKEY]->input(rcData[KEYBOARD_B_CHANNEL]);

        if (RoboModeSwitch_KEY[FOLLOWMODEKEY]->posEdge()) {
            RobotMode = RFOLLOWMODE;
            SwitchRobotMode();
        }
        if (RoboModeSwitch_KEY[RSPINMODEKEY]->posEdge()) {
            RobotMode = RMANUALMODE;
            SwitchRobotMode();
        }
        if (RoboModeSwitch_KEY[MANUALMODEKEY]->posEdge()) {
            RobotMode = RSPINMODE;
            SwitchRobotMode();
        }


        osDelay(1);
    }
}