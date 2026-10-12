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

#include <cstring>

#include "protocol.h"
#include "rc.h"
#include "sys.h"
#include "user_interface.h"
#include "utils.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "referee_task.h"
#include "robot_state.h"
#include "shoot_task.h"

osThreadId_t UITaskHandle;

namespace {
void ClearUiGraphics();
}
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
communication::StringGUI* initGUI = nullptr;

void UI_Delay(uint32_t delay) {
    osDelay(delay);
}

void UITask(void* arg) {
    UNUSED(arg);
    osDelay(300);

    RefereeSnapshot_t referee;
    do {
        referee = RefereeGetSnapshot();
        if (!referee.online || referee.robot_id == 0) osDelay(100);
    } while (!referee.online || referee.robot_id == 0);
    UI->SetID(referee.robot_id);
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
    initGUI->InitString();
    osDelay(110);

    float relative_angle = 0;
    float pitch_angle = 0;

    RemoteMode_e last_mode = REMOTE_MODE_KILL;
    bool last_fric_mode = false;
    ShootMode_e last_shoot_frequency = SHOOT_MODE_IDLE;
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
    uint32_t last_ui_sequence = RCGetSnapshot().ui_sequence;
    while (true) {
        const RemoteControl_t remote_input = RCGetSnapshot();
        if (remote_input.ui_sequence != last_ui_sequence) {
            last_ui_sequence = remote_input.ui_sequence;
            if (remote_input.ui_command == REMOTE_UI_CLEAR_GRAPHICS) {
                ClearUiGraphics();
            } else if (remote_input.ui_command == REMOTE_UI_CLEAR_DIAGNOSTICS &&
                       diagGUI != nullptr) {
                diagGUI->Clear(UI_Delay);
            }
        }

        const GimbalTelemetry_t gimbal = GimbalGetTelemetry();
        const ChassisTelemetry_t chassis = ChassisGetTelemetry();
        const ShooterTelemetry_t shooter = ShooterGetTelemetry();
        const ImuData_t imu = ImuGetData();
        const RemoteMode_e mode = remote_mode;

        relative_angle = gimbal.yaw_relative;
        pitch_angle = gimbal.pitch_relative;
        const float normalized_vx = chassis.max_xy_speed > 0.0f
                                        ? chassis.vx / chassis.max_xy_speed
                                        : 0.0f;
        const float normalized_vy = chassis.max_xy_speed > 0.0f
                                        ? chassis.vy / chassis.max_xy_speed
                                        : 0.0f;
        chassisGUI->Update(normalized_vx, normalized_vy, relative_angle);
        osDelay(UI_OS_DELAY);

        // Chassis-bus voltage/current are sampled on the lower board.
        // Do not fabricate an upper-board battery percentage here.
        // Keep the legacy battery widget unchanged until lower-board telemetry
        // is explicitly bridged back to the upper board.
        osDelay(UI_OS_DELAY);

        // 更新云台小坦克
        gimbalGUI->Update(gimbal.pitch_target * 200, -gimbal.yaw_target * 200, pitch_angle, relative_angle, true);
        osDelay(UI_OS_DELAY);

        char initstr[15];
        if (!gimbal.initialized) {
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
        if (mode != last_mode) {
            char modeStr[30];
            switch (mode) {
                case REMOTE_MODE_KILL:
                    strcpy(modeStr, "STOP MODE     ");
                    modeColor = UI_Color_Purplish_red;
                    break;
                case REMOTE_MODE_FOLLOW:
                    strcpy(modeStr, "FOLLOW MODE   ");
                    modeColor = UI_Color_Green;
                    break;
                case REMOTE_MODE_SPIN:
                    strcpy(modeStr, "SPIN MODE     ");
                    modeColor = UI_Color_Orange;
                    break;
                case REMOTE_MODE_MANUAL:
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
        if (last_fric_mode != shooter.friction_enabled) {
            char* wheelStr = shooter.friction_enabled ? wheelOnStr : wheelOffStr;
            uint32_t wheelColor = shooter.friction_enabled ? UI_Color_Pink : UI_Color_Green;
            wheelGUI->Update(wheelStr, wheelColor);
            osDelay(UI_OS_DELAY);
            last_fric_mode = shooter.friction_enabled;
        }

        // Update shoot frequency status GUI
        if (shooter.mode != last_shoot_frequency) {
            // 统一字符串长度为12字符（与滚轮字符串对齐），保证UI显示整齐
            char shoot_frequency_str[30] = {0};

            switch (shooter.mode) {
                case SHOOT_MODE_UNLOAD:
                    strcpy(shoot_frequency_str, "SUNLOAD    ");  // 退弹模式（12字符）
                    ShootFrequencyColor = UI_Color_Yellow;      // 黄色标识退弹状态
                    break;
                case SHOOT_MODE_IDLE:
                    strcpy(shoot_frequency_str, "STANDBY    ");  // 待机模式（12字符）
                    ShootFrequencyColor = UI_Color_Green;       // 绿色标识正常待机
                    break;
                case SHOOT_MODE_SINGLE:
                    strcpy(shoot_frequency_str, "SINGLESHOT ");  // 单发模式（12字符）
                    ShootFrequencyColor = UI_Color_Green;       // 绿色标识常规模式
                    break;
                case SHOOT_MODE_DOUBLE:
                    strcpy(shoot_frequency_str, "DOUBLETAP  ");  // 连发模式（12字符）
                    ShootFrequencyColor = UI_Color_Orange;      // 橙色标识中速模式
                    break;
                case SHOOT_MODE_RAPID:
                    strcpy(shoot_frequency_str, "RAPIDFIRE  ");  // 速射模式（12字符）
                    ShootFrequencyColor = UI_Color_Purplish_red;// 紫红色标识高速模式
                    break;
                default:
                    strcpy(shoot_frequency_str, "UNKNOWN    ");  // 未知模式（12字符）
                    ShootFrequencyColor = UI_Color_Purplish_red;
                    break;
            }
            last_shoot_frequency = shooter.mode;
            shootFrequencyGUI->Update(shoot_frequency_str, ShootFrequencyColor);
            osDelay(UI_OS_DELAY);
        }

        bool is_autoaim = mode == REMOTE_MODE_AUTOAIM;
        autoAimEdgeDetector->input(is_autoaim);
        if (autoAimEdgeDetector->edge()) {
            char* autoaimStr = is_autoaim ? autoAimStr : autoAimOffStr;
            autoAimGUI->Update(autoaimStr, UI_Color_Pink);
            osDelay(UI_OS_DELAY);
        }
        last_mode = mode;
        last_fric_mode = shooter.friction_enabled;
        last_shoot_frequency = shooter.mode;

        // 离线信息
        {
            fl_motor_check_edge->input(true);
            fr_motor_check_edge->input(true);
            bl_motor_check_edge->input(true);
            br_motor_check_edge->input(true);
            yaw_motor_check_edge->input(gimbal.yaw_online);
            pitch_motor_check_edge->input(gimbal.pitch_online);
            steer_motor_check_edge->input(shooter.feeder_online);
            dbus_edge->input(RCGetSnapshot().online);
            imu_cali_edge->input(ImuCalibrationDone());
            imu_temp_edge->input(imu.online &&
                                  (imu.temperature < 35.0f || imu.temperature > 60.0f));

            // todo:应该是在UI中显示M2006的数据
            shoot_jam_edge->input(shooter.jammed);

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

void InitUI() {
    UI = RefereeCreateUserInterface();
    RM_ASSERT_TRUE(UI != nullptr, "UI init failed");
}

namespace {
void ClearUiGraphics() {
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
}
