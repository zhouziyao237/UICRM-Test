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

#pragma once

#include "bsp_thread.h"
#include "main.h"
#include "utils.h"
#include "referee_task.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"

// ----------------------------- sys start data -----------------------------

typedef struct SysStartData_s {
    uint8_t SystemInitFlag : 1;
    uint8_t RefereeInitFlag : 1;
    uint8_t ImuInitFlag : 1;
    uint8_t GimbalInitFlag : 1;
    uint8_t ChassisInitFlag : 1;
    uint8_t VisionInitFlag : 1;
    uint8_t ShootInitFlag : 1;
    uint8_t AHRSInitFlag : 1;
} SysStartData_t;
extern volatile SysStartData_t SysStartData;

typedef struct {
    uint8_t RemoteOnline : 1;
    uint8_t GimbalOnline : 1;
    uint8_t ChassisOnline : 1;
    uint8_t VisionOnline : 1;
    uint8_t RefereeOnline : 1;
    uint8_t ShootOnline : 1;
    uint8_t RCOnline : 1;
} RobotStatus_t;
extern volatile  RobotStatus_t RobotStatus;

/**
 * @brief 云台控制模式枚举
 *
 * 定义云台系统的不同工作模式，用于控制系统行为
 */
enum GimbalMode_e {
    GKILLMODE = 0,      /**< 急停模式 - 系统紧急停止 */
    GMANUALMODE = 1,    /**< 手动模式 - 人工控制云台 */
    GFOLLOWMODE = 2,    /**< 跟随模式 - 跟随底盘移动 */
    GAUTOAIMODE = 3,    /**< 自瞄模式 - 自动瞄准目标 */
    GSPINMODE = 4,      /**< 旋转模式 - 云台旋转扫射 */
};

/**
 * @brief 云台模式全局变量声明
 *
 * 用于在程序中引用当前云台工作模式
 */
extern volatile GimbalMode_e GimbalMode;

/**
 * @brief 底盘控制模式枚举
 *
 * 定义底盘系统的不同工作模式，控制机器人移动行为
 */
enum ChassisMode_e {
    CKILLMODE = 0,      /**< 急停模式 - 底盘紧急停止 */
    CMANUALMODE = 1,    /**< 手动模式 - 人工控制移动 */
    CFOLLOWMODE = 2,    /**< 跟随模式 - 跟随操作手指令 */
    CAUTOAIMODE = 3,    /**< 自瞄模式 - 配合云台自动瞄准 */
    CSPINMODE = 4,      /**< 旋转模式 - 底盘旋转扫射 */
};

/**
 * @brief 底盘模式全局变量声明
 *
 * 用于在程序中引用当前底盘工作模式
 */
extern volatile ChassisMode_e ChassisMode;

/**
 * @brief 射击控制模式枚举
 *
 * 定义射击系统的不同工作模式，控制发射行为
 */
enum ShootMode_e {
    SUNLOADMODE = -1,
    SKILLMODE = 0,      /**< 急停模式 - 射击系统停止 */
    SHOOTILDE = 1,      /**< 待机模式 - 射击系统准备状态 */
    SINGLESHOT = 2,     /**< 单发模式 - 单次发射弹丸 */
    DOUBLETAP = 3,      /**< 连发模式 - 快速连续发射 */
    RAPIDFIRE = 4,      /**< 速射模式 - 持续高速发射 */
};

/**
 * @brief 射击模式全局变量声明
 *
 * 用于在程序中引用当前射击工作模式
 */
extern volatile ShootMode_e ShootMode;

/**
 * @brief 机器人整体控制模式枚举
 *
 * 定义机器人系统的最高级控制模式，协调各子系统工作
 */
enum RobotMode_e {
    RKILLMODE = 0,      /**< 急停模式 - 整机紧急停止 */
    RMANUALMODE = 1,    /**< 手动模式 - 全手动控制 */
    RSPINMODE = 2,      /**< 旋转模式 - 小陀螺模式 */
    RFOLLOWMODE = 3,    /**< 跟随模式 - 跟随操作 */
    RAUTOAIMODE = 4,     /**< 自瞄模式 - 自动瞄准攻击 */
    RAUTOMODE = 5,      /**< 自动模式 - 全自动运行 */
};

/**
 * @brief 机器人模式全局变量声明
 *
 * 用于在程序中引用当前机器人整体工作模式
 */
extern volatile RobotMode_e RobotMode;


// ----------------------------- sys Hardware Code -----------------------------

// ----------------------------- CAN Code -----------------------------

#include "bsp_can.h"

extern bsp::CAN* can1;
extern bsp::CAN* can2;

void InitCAN();

// ----------------------------- UART Code -----------------------------
#include "bsp_uart.h"

#define RefereePort huart1
#define ENABLE_UI false
#define HAS_REFEREE

#define RCPort huart6
#define RCPortBaudrate 912600
// #define RCPortTXBUFFERSIZE 100
#define RCPortRXBUFFERSIZE 300

extern bsp::UART* referee_uart;
extern bsp::UART* rc_uart;

void InitRefereeUart();
void InitRCUart();

// ----------------------------- USB Code -----------------------------
#include "bsp_usb.h"

extern bsp::VirtualUSB* minipc_usb;

void InitMinipcUSB();

// ----------------------------- LOG Code -----------------------------
#include "bsp_print.h"

#define LOGPort rc_uart

void InitLOGPort();

// ----------------------------- SPI Code -----------------------------
#include "bsp_spi.h"

#define HSPI1_Prot 0
#define HSPI1CFG {&hspi1, bsp::SPI_MODE_DMA}

#define HSPI_NUM 1
extern bsp::spi_init_t SPICFGS[HSPI_NUM];
extern bsp::SPI* SPIS[HSPI_NUM];

void InitSPIPort();

// ----------------------------- I2C Code -----------------------------
#include "bsp_i2c.h"

#define I2C3_Prot 0
#define I2C3CFG {&hi2c3, bsp::I2C_MODE_IT}

#define I2C_NUM 1
extern bsp::i2c_init_t I2CCFGS[I2C_NUM];
extern bsp::I2C* I2CS[I2C_NUM];

void InitI2CPort();

// ----------------------------- GPIO Code -----------------------------
#include "bsp_gpio.h"

#define BMI088_ACCEL_CS_Pin 0
#define BMI_GRYO_CS_Pin 1
#define IST8310_RST_Pin 2

#define GPIO_NUM 3
extern GPIO_TypeDef* group[GPIO_NUM];
extern uint16_t pin[GPIO_NUM];

extern bsp::GPIO* GPIOS[GPIO_NUM];

void InitGPIO();

// ----------------------------- INT GPIO Code -----------------------------
#define BMI088_ACCEL_INT_Pin 0
#define BMI088_GRYO_INT_Pin 1
#define IST8310_INT_Pin 2

#define INT_GPIO_NUM 3
extern uint16_t intpin[INT_GPIO_NUM];

extern bsp::GPIT* INTGPIOS[INT_GPIO_NUM];

void InitINTGPIO();

// ----------------------------- PWM GPIO Code -----------------------------

#include "bsp_pwm.h"

typedef struct {
    TIM_HandleTypeDef* htim;  ///< HAL定时器句柄
    uint8_t channel;          ///< 定时器通道，可选[1,2,3,4]
    uint32_t clock_freq;      ///< 定时器时钟频率，单位为Hz
    uint32_t output_freq;     ///< 输出频率，单位为Hz
    uint32_t pulse_width;     ///< 输出脉宽，单位为us
} PWMCFG_t;

#define heater_pwm 0
#define heater_pwm_cfg {&htim10, 1, 1000000, 2000, 0}

#define PWM_GPIO_NUM 1

extern PWMCFG_t PWMCFG[PWM_GPIO_NUM];
extern bsp::PWM* PWMS[PWM_GPIO_NUM];

void InitPWMGPIO();

// -------------------------- sys Software Code --------------------------

// 通道名称与变量映射
#define RC_CHANNEL_ROLL ROLL      // 横滚通道
#define RC_CHANNEL_PITCH PITCH     // 俯仰通道
#define RC_CHANNEL_YAW YAW       // 偏航通道
#define RC_CHANNEL_THROTTLE THROTTLE  // 油门通道
#define RC_CHANNEL_AUX1 AUX1      // 辅助通道1
#define RC_CHANNEL_AUX2 AUX2      // 辅助通道2 (扳机)
#define RC_CHANNEL_AUX3 AUX3      // 辅助通道3
#define RC_CHANNEL_AUX4 AUX4      // 辅助通道4 (暂停按钮)
#define RC_CHANNEL_AUX5 AUX5      // 辅助通道5 (左键自定义)
#define RC_CHANNEL_AUX6 AUX6      // 辅助通道6 (右键自定义)
#define RC_CHANNEL_AUX7 AUX7     // 辅助通道7
#define RC_CHANNEL_AUX8 AUX8     // 辅助通道8
#define RC_CHANNEL_AUX9 AUX9     // 辅助通道9
#define RC_CHANNEL_AUX10 AUX10    // 辅助通道10 (鼠标左键)
#define RC_CHANNEL_AUX11 AUX11    // 辅助通道11 (鼠标右键)
#define RC_CHANNEL_AUX12 AUX12    // 辅助通道12 (鼠标中键)

// 按键通道快捷映射
#define TRIGGER_CHANNEL RC_CHANNEL_AUX2      // 扳机通道
#define PAUSE_BUTTON_CHANNEL RC_CHANNEL_AUX4 // 暂停按钮通道
#define CUSTOM_LEFT_CHANNEL RC_CHANNEL_AUX5  // 左键自定义通道
#define CUSTOM_RIGHT_CHANNEL RC_CHANNEL_AUX6 // 右键自定义通道
#define MOUSE_LEFT_CHANNEL RC_CHANNEL_AUX10  // 鼠标左键通道
#define MOUSE_RIGHT_CHANNEL RC_CHANNEL_AUX11 // 鼠标右键通道
#define MOUSE_MIDDLE_CHANNEL RC_CHANNEL_AUX12 // 鼠标中键通道

// 键盘按键通道映射 (AUX13-AUX22)
#define KEYBOARD_W_CHANNEL      AUX13    // W键 - 前进
#define KEYBOARD_S_CHANNEL      AUX14    // S键 - 后退
#define KEYBOARD_A_CHANNEL      AUX15    // A键 - 左移
#define KEYBOARD_D_CHANNEL      AUX16    // D键 - 右移
#define KEYBOARD_SHIFT_CHANNEL  AUX17    // SHIFT键 - 加速
#define KEYBOARD_CTRL_CHANNEL   AUX18    // CTRL键 - 蹲下
#define KEYBOARD_Q_CHANNEL      AUX19    // Q键 - 左转
#define KEYBOARD_E_CHANNEL      AUX20    // E键 - 右转
#define KEYBOARD_R_CHANNEL      AUX21    // R键 - 补给
#define KEYBOARD_F_CHANNEL      AUX22    // F键 - 功能键
#define KEYBOARD_G_CHANNEL      AUX23    // G键 - 模式顺序切换
#define KEYBOARD_Z_CHANNEL      AUX24    // Z键 - 摩擦轮模式切换
#define KEYBOARD_X_CHANNEL      AUX25    // X键 - 清理UI消息

#define KEYBOARD_C_CHANNEL      AUX26
#define KEYBOARD_V_CHANNEL      AUX27
#define KEYBOARD_B_CHANNEL      AUX28

#define CLEARGUIMSGKEY KEYBOARD_X_CHANNEL

// 建议添加的宏定义
#define CH_FRIC_ONOFF           CUSTOM_LEFT_CHANNEL
#define KEYBORAD_FRIC_ONOFF     KEYBOARD_Z_CHANNEL
#define CH_UNLOAD               PAUSE_BUTTON_CHANNEL
#define KEYBORAD_UNLOAD         KEYBOARD_R_CHANNEL

#define CFORWARD KEYBOARD_W_CHANNEL
#define CLEFT KEYBOARD_A_CHANNEL
#define CBACKWARD KEYBOARD_S_CHANNEL
#define CRIGHT KEYBOARD_D_CHANNEL
#define CROTATELEFT KEYBOARD_Q_CHANNEL
#define CROTATERIGHT KEYBOARD_E_CHANNEL
#define CSPEEDUP KEYBOARD_SHIFT_CHANNEL
#define CSPEEDDOWN KEYBOARD_CTRL_CHANNEL

#define mouse_ratio 0.8
#define remote_ratio 0.005
#define MOUSE_PitchInverted true
#define MOUSE_YawInverted false

#define CH_PitchInverted true
#define CH_YawInverted false

#define GIMBAL_PITCH_CH YAW
#define GIMBAL_YAW_CH THROTTLE
#define GIMBAL_MOUSE_X_CH MOUSE_Y_AXIS
#define GIMBAL_MOUSE_Y_CH MOUSE_X_AXIS


#define CHASSIS_FB_CH      RC_CHANNEL_ROLL   // 前进后退通道
#define CHASSIS_LR_CH      RC_CHANNEL_PITCH  // 左右平移通道
#define CHASSIS_ROT_CH     RC_CHANNEL_AUX1   // 旋转通道

#define CHASSIS_FB_REV     true
#define CHASSIS_LR_REV     true
#define CHASSIS_ROT_REV    true

#define ROBOMODE_SWITCH_CH CUSTOM_RIGHT_CHANNEL
#define ROBOMODE_SWITCH_KEY KEYBOARD_G_CHANNEL

// 模式选择开关通道（三档开关）
#define MODE_SWITCH_CHANNEL RC_CHANNEL_AUX3  // 模式选择开关（UP/MID/DOWN）


// 标准按键配置格式：{通道号, 按下阈值, 释放阈值, 长按时间(ms), 双击超时(ms)}
#define RCKEYSNUM 3
#define CONFIGS {TRIGGERKEYCONFIG, MOUSELEFTCONFIG, CLEARGUIMSGKEYCONFIG}
#define TRIGGERKEYCONFIG {TRIGGER_CHANNEL, 0, 1, 500, 150}     // 扳机 trigger
#define CLEARGUIMSGKEYCONFIG {KEYBOARD_X_CHANNEL, 0, 1, 500, 200}       // 清理 UI
// #define CUSTOMLEFTKEYCONFIG {CUSTOM_LEFT_CHANNEL, 1, 0, 1500, 200}  // 左键自定义按键 custom_left
// #define CUSTOMRIGHTKEYCONFIG {CUSTOM_RIGHT_CHANNEL, 1, 0, 800, 200}  // 右键自定义按键 custom_right

// 鼠标按键配置
#define MOUSELEFTCONFIG {MOUSE_LEFT_CHANNEL, 0, 1, 500, 150}      // 鼠标左键 mouse_left
// #define MOUSERIGHTCONFIG {MOUSE_RIGHT_CHANNEL, 1, 0, 500, 150}     // 鼠标右键 mouse_right
// #define MOUSEMIDDLECONFIG {MOUSE_MIDDLE_CHANNEL, 1, 0, 1000, 150}   // 鼠标中键 mouse_middle

// 双重冗余安全检查：VTM 硬件层 (100ms) + 应用层去抖 (200ms)
inline uint32_t rc_offline_start_time = 0;
inline uint32_t APP_OFFLINE_DEBOUNCE_MS = 200;  // 应用层额外 200ms 去抖

// ------------------------- sys Task Data Code --------------------------

enum SWITCHMode {
    MODE_KILL = 1,
    MODE_RUN = 2,
    NONE = 3,
};

enum ModeSwitchKEY {
    SwitchKEY = 1,
    FOLLOWMODEKEY = 2,
    RSPINMODEKEY = 3,
    MANUALMODEKEY = 4,

    SwitchKEYNUM
};

extern bool rc_offline;
extern bool is_robot_dead;

extern BoolEdgeDetector* RoboModeSwitch_CH;
extern BoolEdgeDetector* RoboModeSwitch_KEY[SwitchKEYNUM];

// ---------------------------- sys Task Code ----------------------------

extern osThreadId_t SysTaskHandle;
const osThreadAttr_t SysTaskAttribute = {
    .name = "SysTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityHigh5,
    .tz_module = 0,
    .reserved = 0};

// ----------------------------- sys Main Code -----------------------------

void InitRMSys();
void RoboSysInit();
void SwitchRobotMode();
void SysTask(void* arg);