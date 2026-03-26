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

#include "ui_task.h"

#include "gimbal_task.h"
#include "referee_task.h"
#include "shoot_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "imu_task.h"

osThreadId_t uiTaskHandle;
communication::UserInterface* UI = nullptr;
communication::ChassisGUI* chassisGUI = nullptr;
communication::CrossairGUI* crossairGui = nullptr;
communication::GimbalGUI* gimbalGUI = nullptr;
communication::CapGUI* batteryGUI = nullptr;
communication::StringGUI* modeGUI = nullptr;
communication::StringGUI* wheelGUI = nullptr;
communication::StringGUI* shootFrequencyGUI = nullptr;
communication::StringGUI* boostGUI = nullptr;
communication::StringGUI* autoAimGUI = nullptr;
communication::DiagGUI* diagGUI = nullptr;
communication::StringGUI* BombchamberGUI = nullptr;
communication::StringGUI* initGUI = nullptr;

void UI_Delay(uint32_t delay) {
    osDelay(delay);
}

void uiTask(void* arg) {
    UNUSED(arg);
    osDelay(300);

    while (!SysStartData.SystemInitFlag) {
        osDelay(10);
    }

    UI->SetID(Referee_data->game_robot_status.robot_id);
    osDelay(UI_OS_DELAY);

    communication::graphic_data_t graphEmpty1;
    UI->CircleDraw(&graphEmpty1, "E1", UI_Graph_Del, 0, UI_Color_Green, 0, 0, 0, 0);
    communication::graphic_data_t graphEmpty2;
    UI->CircleDraw(&graphEmpty2, "E2", UI_Graph_Del, 0, UI_Color_Green, 0, 0, 0, 0);

    // Initialize chassis_main GUI
    chassisGUI = new communication::ChassisGUI(UI);
    osDelay(110);
    chassisGUI->Init2();
    osDelay(110);
    // Initialize crosshair GUI
    crossairGui = new communication::CrossairGUI(UI);
    osDelay(110);

    // Initialize chassis power GUI
    char batteryStr[15] = "CHASSIS_POWER";
    batteryGUI = new communication::CapGUI(UI, batteryStr);
    osDelay(110);
    batteryGUI->InitName();
    osDelay(110);


    // Initialize Gimbal GUI
    gimbalGUI = new communication::GimbalGUI(UI);
    osDelay(110);
    gimbalGUI->Init2();
    osDelay(110);

    // Initialize self-diagnosis GUI
    char diagStr[29] = "";
    diagGUI = new communication::DiagGUI(UI);

    char initModeStr[15] = "Init False!";
    initGUI = new communication::StringGUI(UI, initModeStr, 1500, 520, UI_Color_Pink);

    // Initialize current mode GUI
    char followModeStr[15] = "FOLLOW MODE";
    int8_t modeColor = UI_Color_Orange;
    modeGUI = new communication::StringGUI(UI, followModeStr, 810, 120, modeColor, 30);
    // Initialize flywheel status GUI
    char wheelOnStr[15] = "FLYWHEEL ON";
    char wheelOffStr[15] = "FLYWHEEL OFF";
    wheelGUI = new communication::StringGUI(UI, wheelOffStr, 1500, 430, UI_Color_Pink);
    char boostOffStr[15] = " ";
    boostGUI = new communication::StringGUI(UI, boostOffStr, 870, 630, UI_Color_Pink, 30);

    char autoAimStr[15] = "AUTOAIM ";
    char autoAimOffStr[15] = "        ";
    autoAimGUI = new communication::StringGUI(UI, autoAimOffStr, 840, 730, UI_Color_Orange, 30);

    // Initialize current mode GUI
    char ShootFrequencyStr[15] = "NORMAL";
    int8_t ShootFrequencyColor = UI_Color_Green;
    shootFrequencyGUI = new communication::StringGUI(UI, ShootFrequencyStr, 1500, 460, ShootFrequencyColor);

    char BombchamberOnStr[15] = "MagazineON ";
    BombchamberGUI = new communication::StringGUI(UI, BombchamberOnStr, 1500, 490, UI_Color_Orange);

    modeGUI->Init();
    osDelay(110);
    wheelGUI->Init();
    osDelay(110);
    boostGUI->Init();
    osDelay(110);
    autoAimGUI->Init();
    osDelay(110);
    shootFrequencyGUI->Init();
    osDelay(110);
    BombchamberGUI->Init();
    osDelay(110);
    initGUI->Init();

    osDelay(110);
    modeGUI->InitString();
    osDelay(110);
    wheelGUI->InitString();
    osDelay(110);
    boostGUI->InitString();
    osDelay(110);
    autoAimGUI->InitString();
    osDelay(110);
    shootFrequencyGUI->InitString();
    osDelay(110);
    BombchamberGUI->InitString();
    osDelay(110);
    initGUI->InitString();
    osDelay(110);

    float relative_angle = 0;
    float pitch_angle = 0;

    float power_percent = 1;
    int8_t last_mode = RKILLMODE;
    uint8_t last_fric_mode = FW_MODE_OFF;
    uint8_t last_shoot_frequency = SKILLMODE;
    BoolEdgeDetector* autoAimEdgeDetector = new BoolEdgeDetector(false);

    BoolEdgeDetector* fl_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* fr_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* bl_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* br_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* yaw_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* pitch_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* steer_motor_check_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* dbus_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* imu_cali_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* imu_temp_edge = new BoolEdgeDetector(false);
    BoolEdgeDetector* shoot_jam_edge = new BoolEdgeDetector(false);
    while (true) {
        // Update chassis_main GUI
        // 通过两个云台电机的角度
        relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);
        pitch_angle = pitch_motor->GetThetaDelta(gimbal_param->pitch_offset_);
        // 更新底盘转速码表
        chassisGUI->Update(ChassisMotion.chassis_vx / chassis_vx_max, ChassisMotion.chassis_vy / chassis_vy_max,
                           relative_angle);
        osDelay(UI_OS_DELAY);

        // 更新电源百分比
        power_percent = battery_vol->GetBatteryPercentage();
        batteryGUI->Update(power_percent);
        osDelay(UI_OS_DELAY);

        // 更新云台小坦克
        gimbalGUI->Update(gimbal->getPitchTarget() * 200, -gimbal->getYawTarget() * 200, pitch_angle, relative_angle, true);
        osDelay(UI_OS_DELAY);

        char initstr[15];
        if (!SysStartData.GimbalInitFlag) {
            strcpy(initstr, "INIT False");
            modeColor = UI_Color_Purplish_red;
            initGUI->Update(initstr, modeColor);
        }else {
            strcpy(initstr, "INIT Ture");
            modeColor = UI_Color_Green;
            initGUI->Update(initstr, modeColor);
        }
        osDelay(UI_OS_DELAY);

        // 更新运行模式
        if (RobotMode != last_mode) {
            char modeStr[30];
            switch (RobotMode) {
                case RKILLMODE:
                    strcpy(modeStr, "STOP MODE     ");
                    modeColor = UI_Color_Purplish_red;
                    break;
                case RFOLLOWMODE:
                    strcpy(modeStr, "FOLLOW MODE   ");
                    modeColor = UI_Color_Green;
                    break;
                case RSPINMODE:
                    strcpy(modeStr, "SPIN MODE     ");
                    modeColor = UI_Color_Orange;
                    break;
                case RMANUALMODE:
                    strcpy(modeStr, "ADVANCED MODE ");
                    modeColor = UI_Color_Green;
                    break;
                default:
                    strcpy(modeStr, "UNKNOWN MODE   ");
                    modeColor = UI_Color_Purplish_red;
                    break;
            }

            modeGUI->Update(modeStr, modeColor);
            osDelay(UI_OS_DELAY);
        }

        // Update wheel status GUI
        if (last_fric_mode != FWMode) {
            char* wheelStr = FWMode == FW_MODE_ON ? wheelOnStr : wheelOffStr;
            uint32_t wheelColor = FWMode == FW_MODE_ON ? UI_Color_Pink : UI_Color_Green;
            wheelGUI->Update(wheelStr, wheelColor);
            osDelay(UI_OS_DELAY);
            last_fric_mode = FWMode;  // 补充滚轮模式同步，避免重复更新
        }

        // Update shoot frequency status GUI
        if (ShootMode != last_shoot_frequency) {
            // 统一字符串长度为12字符（与滚轮字符串对齐），保证UI显示整齐
            char shoot_frequency_str[30] = {0};

            switch (ShootMode) {
                case SUNLOADMODE:
                    strcpy(shoot_frequency_str, "SUNLOAD    ");  // 退弹模式（12字符）
                    ShootFrequencyColor = UI_Color_Yellow;      // 黄色标识退弹状态
                    break;
                case SKILLMODE:
                    strcpy(shoot_frequency_str, "SKILL      ");  // 急停模式（12字符）
                    ShootFrequencyColor = UI_Color_Purplish_red;// 紫红色标识停止状态
                    break;
                case SHOOTILDE:
                    strcpy(shoot_frequency_str, "STANDBY    ");  // 待机模式（12字符）
                    ShootFrequencyColor = UI_Color_Green;       // 绿色标识正常待机
                    break;
                case SINGLESHOT:
                    strcpy(shoot_frequency_str, "SINGLESHOT ");  // 单发模式（12字符）
                    ShootFrequencyColor = UI_Color_Green;       // 绿色标识常规模式
                    break;
                case DOUBLETAP:
                    strcpy(shoot_frequency_str, "DOUBLETAP  ");  // 连发模式（12字符）
                    ShootFrequencyColor = UI_Color_Orange;      // 橙色标识中速模式
                    break;
                case RAPIDFIRE:
                    strcpy(shoot_frequency_str, "RAPIDFIRE  ");  // 速射模式（12字符）
                    ShootFrequencyColor = UI_Color_Purplish_red;// 紫红色标识高速模式
                    break;
                default:
                    strcpy(shoot_frequency_str, "UNKNOWN    ");  // 未知模式（12字符）
                    ShootFrequencyColor = UI_Color_Purplish_red;
                    break;
            }
            last_shoot_frequency = ShootMode;
            shootFrequencyGUI->Update(shoot_frequency_str, ShootFrequencyColor);
            osDelay(UI_OS_DELAY);
        }

        bool is_autoaim = GimbalMode == GAUTOAIMODE;
        autoAimEdgeDetector->input(is_autoaim);
        if (autoAimEdgeDetector->edge()) {
            char* autoaimStr = is_autoaim ? autoAimStr : autoAimOffStr;
            autoAimGUI->Update(autoaimStr, UI_Color_Pink);
            osDelay(UI_OS_DELAY);
        }
        last_mode = RobotMode;
        last_fric_mode = FWMode;
        last_shoot_frequency = ShootMode;

        // 离线信息
        {
            fl_motor_check_edge->input(true);
            fr_motor_check_edge->input(true);
            bl_motor_check_edge->input(true);
            br_motor_check_edge->input(true);
            yaw_motor_check_edge->input(yaw_motor->IsOnline());
            pitch_motor_check_edge->input(pitch_motor->IsOnline());
            steer_motor_check_edge->input(FeedingMotror->IsOnline());
            dbus_edge->input(RCIsOnline());
            imu_cali_edge->input(ahrs->IsCailbrated());
            imu_temp_edge->input(true);

            // todo:应该是在UI中显示M2006的数据
            shoot_jam_edge->input(ShootMode == SUNLOADMODE);

            if (fl_motor_check_edge->negEdge()) {
                strcpy(diagStr, "FL MOTOR OFFLINE     ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (fr_motor_check_edge->negEdge()) {
                strcpy(diagStr, "FR MOTOR OFFLINE     ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (bl_motor_check_edge->negEdge()) {
                strcpy(diagStr, "BL MOTOR OFFLINE     ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (br_motor_check_edge->negEdge()) {
                strcpy(diagStr, "BR MOTOR OFFLINE     ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (yaw_motor_check_edge->negEdge()) {
                strcpy(diagStr, "YAW MOTOR OFFLINE    ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (pitch_motor_check_edge->negEdge()) {
                strcpy(diagStr, "PITCH MOTOR OFFLINE  ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (steer_motor_check_edge->negEdge()) {
                strcpy(diagStr, "STEER MOTOR OFFLINE  ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (dbus_edge->negEdge()) {
                strcpy(diagStr, "DBUS OFFLINE         ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
            if (imu_cali_edge->posEdge()) {
                strcpy(diagStr, "IMU CALIBRATION DONE");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Green);
            }
            if (imu_temp_edge->posEdge()) {
                strcpy(diagStr, "IMU TEMP NOT SAFE   ");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }

            // todo: 显示堵转检测
            if (shoot_jam_edge->posEdge()) {
                strcpy(diagStr, "STEER JAM");
                diagGUI->Update(diagStr, UI_Delay, UI_Color_Pink);
            }
        }

    }
}

void UI_callback(uint8_t channel, rc_key_mode_t mode) {
    if (channel == CLEARGUIMSGKEY) {
        switch (mode) {
            case RC_KEY_NONE:
                break;
            case RC_KEY_CLICK: {
                clean_GUIMSG();
            }break;
            case RC_KEY_LONG_PRESS: {
                diagGUI->Clear(UI_Delay);
            }break;
            default:
                break;
        }
    }
}

void InitUI() {
    UI = new communication::UserInterface(referee_uart, Referee_data);
    rc_key_add_callback(UI_callback);
}

void clean_GUIMSG() {
    osDelay(110);
    chassisGUI->Delete2();
    osDelay(110);
    chassisGUI->Delete();
    osDelay(110);
    gimbalGUI->Delete2();
    osDelay(110);
    gimbalGUI->Delete();
    osDelay(110);
    crossairGui->Delete();
    osDelay(110);
    batteryGUI->DeleteName();
    osDelay(110);
    batteryGUI->Delete();
    osDelay(110);
    modeGUI->Delete();
    osDelay(110);
    wheelGUI->Delete();
    osDelay(110);
    boostGUI->Delete();
    osDelay(110);
    autoAimGUI->Delete();
    osDelay(110);
    shootFrequencyGUI->Delete();
    osDelay(110);
    diagGUI->Clear(UI_Delay);
    osDelay(110);
    chassisGUI->Init();
    osDelay(110);
    chassisGUI->Init2();
    osDelay(110);
    gimbalGUI->Init();
    osDelay(110);
    gimbalGUI->Init2();
    osDelay(110);
    crossairGui->Init();
    osDelay(110);
    batteryGUI->Init();
    osDelay(110);
    batteryGUI->InitName();
    osDelay(110);
    modeGUI->Init();
    osDelay(110);
    wheelGUI->Init();
    osDelay(110);
    boostGUI->Init();
    osDelay(110);
    shootFrequencyGUI->Init();
    osDelay(110);
    shootFrequencyGUI->InitString();
    osDelay(110);
    modeGUI->InitString();
    osDelay(110);
    wheelGUI->InitString();
    osDelay(110);
    boostGUI->InitString();
    osDelay(110);
    autoAimGUI->Init();
    osDelay(110);
    autoAimGUI->InitString();
    osDelay(110);
}