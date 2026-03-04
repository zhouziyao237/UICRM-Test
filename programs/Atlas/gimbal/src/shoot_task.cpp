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
// Created by Administrator on 2026/3/2.
//

#include "shoot_task.h"

// ----------------------------- Shoot Task code -----------------------------

osThreadId_t ShootTaskHandle;

// ----------------------------- Shoot data code -----------------------------

#define FW_LEFT_MOTOR_ID 0x202
#define FW_RIGHT_MOTOR_ID 0x203
#define FEEDING_MOTOR_ID 0x201

FrictionWheelMode_e FWMode = FW_MODE_OFF;
driver::MotorCANBase* FWMotor[FW_NUM] = {nullptr};
driver::MotorCANBase* FeedingMotror = nullptr;

BoolEdgeDetector* shoot_edge[SEDGENUM] = {nullptr};

StallDetection stall_detector;

// --------------------------- Shoot Callback code ---------------------------

void trigger_callback(uint8_t channel, rc_key_mode_t mode) {
    if (channel == TRIGGER_CHANNEL || channel == MOUSE_LEFT_CHANNEL) {
        if (FWMode == FW_MODE_OFF || ShootMode != SHOOTILDE)
            return;

        switch (mode) {
            case RC_KEY_NONE:
                break;
            case RC_KEY_CLICK: {
                ShootMode = SINGLESHOT;
            }break;

            case RC_KEY_DOUBLE_CLICK:{
                ShootMode = DOUBLETAP;
            }break;

            case RC_KEY_LONG_PRESS: {
                ShootMode = RAPIDFIRE;
            }break;

            default:
                break;
        };

    }
}

void stall_detection_callback(void* instance) {
    driver::Motor2006* motor = static_cast<driver::Motor2006*>(instance);
    stall_detector.Update(motor);
}

void StallCallback(void* instance) {
    UNUSED(instance);
    ShootMode = SUNLOADMODE;
}

// ----------------------------- Shoot Main code -----------------------------

void InitShoot() {
    FWMotor[FW_LEFT] = new driver::Motor3508(can2, FW_LEFT_MOTOR_ID);
    FWMotor[FW_RIGHT] = new driver::Motor3508(can2, FW_RIGHT_MOTOR_ID);

    for (int i = 0; i < FW_NUM; i++) {
        FWMotor[i]->SetTransmissionRatio(19);
        FWMotor[i]->ReInitPID(flywheel_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
        FWMotor[i]->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL);
        FWMotor[i]->SetTarget(0);
        FWMode = FW_MODE_OFF;
    }

    FeedingMotror = new driver::Motor2006(can1, FEEDING_MOTOR_ID);
    FeedingMotror->SetTransmissionRatio(36);
    FeedingMotror->ReInitPID(steering_motor_omega_pid_init,
                             driver::MotorCANBase::SPEED_LOOP_CONTROL);
    FeedingMotror->ReInitPID(steering_motor_theta_pid_init,
                             driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    FeedingMotror->SetMode(driver::MotorCANBase::ANGLE_LOOP_CONTROL |
                           driver::MotorCANBase::SPEED_LOOP_CONTROL);
    FeedingMotror->RegisterDataUpdateCallback(stall_detection_callback, FeedingMotror);
    stall_detector.SetCallback(StallCallback, nullptr);

    for (int i = 0; i < FW_NUM; i++) {
        FWMotor[i]->Disable();
        FWMode = FW_MODE_OFF;
    }

    for (int i = 0; i < SEDGENUM; i++) {
        shoot_edge[i] = new BoolEdgeDetector(false);
    }

    rc_key_add_callback(trigger_callback);
}

void unloadAmmo() {
    FeedingMotror->SetTarget(FeedingMotror->GetTarget() + UNLOAD_FEED_THETA);
    osDelay(200);
    FeedingMotror->SetTarget(FeedingMotror->GetTarget() - UNLOAD_FEED_THETA);
}

void ShootHeatControl() {
#ifdef HAS_REFEREE
    uint8_t game_progress = referee->game_status.game_progress;
    uint16_t game_progress_shifted = game_progress >> 3;
    uint8_t game_progress_bit = game_progress_shifted & 0x01;
    if (game_progress_bit == 4) {
        uint16_t bullet_remaining_num_17mm = referee->bullet_remaining.bullet_remaining_num_17mm;
        if (bullet_remaining_num_17mm == 0) {
            print("not have bullet!\n");
            ShootMode = SHOOTILDE;
        }
    }

    float bullet_speed = referee->shoot_data.bullet_speed;
    if (bullet_speed >= 23.0f) {
        osDelay(SHOOT_OS_DELAY);
        bullet_speed = 0;
    }

    int heat_limit = referee->game_robot_status.shooter_heat_limit;
    int heat_buffer = referee->power_heat_data.shooter_id1_17mm_cooling_heat;
    const int shooter_heat_threashold = 10;
    if (heat_buffer > heat_limit - shooter_heat_threashold) {
        // 临时解决方案
        ShootMode = SHOOTILDE;
        print("overheat!\n");
        osDelay(SHOOT_OS_DELAY);
    }
    UNUSED(heat_limit);
    UNUSED(heat_buffer);
    UNUSED(bullet_speed);

#endif
}

uint8_t ShootIsOnline() {
    return RobotStatus.ShootOnline;
}

void ShootRCUpdata() {
    if (RCIsOnline()) {
        rc_offline_start_time = 0;

        shoot_edge[FEMODE_EDGE_CH]->input((bool)rcData[CH_FRIC_ONOFF]);
        shoot_edge[FEMODE_EDGE_KEYBOARD]->input((bool)rcData[KEYBORAD_FRIC_ONOFF]);
        if (shoot_edge[FEMODE_EDGE_CH]->posEdge() || shoot_edge[FEMODE_EDGE_KEYBOARD]->posEdge()) {
            if (FWMode == FW_MODE_OFF) {
                FWMode = FW_MODE_ON;
            } else if (FWMode == FW_MODE_ON) {
                FWMode = FW_MODE_OFF;
            }
        }

        shoot_edge[CLEAR_FEEDER_CH]->input((bool)rcData[CH_UNLOAD]);
        shoot_edge[CLEAR_FEEDER_KEYBOARD]->input((bool)rcData[KEYBORAD_UNLOAD]);
        if (shoot_edge[CLEAR_FEEDER_CH]->posEdge() ||
            shoot_edge[CLEAR_FEEDER_KEYBOARD]->posEdge()) {
            ShootMode = SUNLOADMODE;
        }

    } else {
        // 离线计时
        if (rc_offline_start_time == 0) {
            rc_offline_start_time = bsp::GetHighresTickMilliSec();
        }

        // 双重确认：VTM(100ms) + 应用层 (200ms) = 总共 300ms
        if (bsp::GetHighresTickMilliSec() - rc_offline_start_time > APP_OFFLINE_DEBOUNCE_MS) {
            ShootMode = SKILLMODE;
            FWMode = FW_MODE_OFF;
        }
        // 否则保持当前模式
    }
}

void ShootTask(void* arg) {
    UNUSED(arg);

    for (int i = 0; i < FW_NUM; i++) {
        if (!FWMotor[i] || !FeedingMotror)
            RM_ASSERT_TRUE(false, "Shoot Init Failed!");
    }

    for (int i = 0; i < FW_NUM; i++) {
        FWMotor[i]->Disable();
    }
    FeedingMotror->Disable();

    bool InitFlag = false;
    while (true) {
        for (int i = 0; i < FW_NUM; i++) {
            if (!FWMotor[i]->IsOnline() || !FeedingMotror->IsOnline()) {
                osDelay(10);
                ShootMode = SKILLMODE;
                FWMode = FW_MODE_OFF;
                RobotStatus.ShootOnline = 0;
                InitFlag = false;
            }
        }
        RobotStatus.ShootOnline = 1;

        if (ShootMode == SKILLMODE) {
            for (int i = 0; i < FW_NUM; i++) {
                FWMotor[i]->Disable();
            }
            FWMode = FW_MODE_OFF;
            FeedingMotror->Disable();
            InitFlag = false;
            osDelay(10);
            continue;
        }

        if (SysStartData.GimbalInitFlag && ShootMode == SHOOTILDE
            && RobotMode != RKILLMODE) {
            InitFlag = true;
            osDelay(10);
        }

        if (InitFlag) {
            if (ShootMode == SKILLMODE) {
                for (int i = 0; i < FW_NUM; i++) {
                    FWMotor[i]->Disable();
                }
                FeedingMotror->Disable();
                SysStartData.ShootInitFlag = 0;
                InitFlag = false;
                continue;
            }
            SysStartData.ShootInitFlag = 1;
            for (int i = 0; i < FW_NUM; i++) {
                FWMotor[i]->Enable();
                FWMotor[i]->SetTarget(0);
            }
            FeedingMotror->Enable();
            break;
        }
        osDelay(100);
    }

    while (true) {
        if (ShootMode == SKILLMODE) {
            while (true) {
                if (ShootMode != SKILLMODE) {
                    FWMode = FW_MODE_OFF;
                    for (int i = 0; i < FW_NUM; i++) {
                        FWMotor[i]->Enable();
                    }
                    FeedingMotror->Enable();
                    osDelay(10);
                    break;
                }
                for (int i = 0; i < FW_NUM; i++) {
                    FWMotor[i]->Disable();
                }
                FeedingMotror->Disable();
                osDelay(10);
            }
        }

        for (int i = 0; i < FW_NUM; i++) {
            if (!FWMotor[i]->IsOnline() || !FeedingMotror->IsOnline()) {
                osDelay(10);
                ShootMode = SKILLMODE;
                FWMode = FW_MODE_OFF;
                RobotStatus.ShootOnline = 0;
            }
        }
        if (RobotStatus.ShootOnline == 0) {
            ShootMode = SKILLMODE;
            FWMode = FW_MODE_OFF;
            for (int i = 0; i < FW_NUM; i++) {
                FWMotor[i]->Disable();
            }
            FeedingMotror->Disable();
            continue;
        }
        RobotStatus.ShootOnline = 1;

        ShootRCUpdata();

        ShootHeatControl();

        switch (FWMode) {
            case FW_MODE_OFF: {
                for (int i = 0; i < FW_NUM; i++) {
                    FWMotor[i]->Disable();
                    FWMotor[i]->SetTarget(0);
                }
            } break;
            case FW_MODE_ON: {
                FWMotor[FW_LEFT]->Enable();
                FWMotor[FW_LEFT]->SetTarget(-FW_SPEED);
                FWMotor[FW_RIGHT]->Enable();
                FWMotor[FW_RIGHT]->SetTarget(FW_SPEED);
            } break;

            default:
                break;
        }

        if (FWMode == FW_MODE_OFF && ShootMode != SUNLOADMODE) {
            ShootMode = SHOOTILDE;
        }

        switch (ShootMode) {
            case SUNLOADMODE: {
                unloadAmmo();
                ShootMode = SHOOTILDE;
            } break;
            case SKILLMODE:
                break;
            case SHOOTILDE:
                break;
            case SINGLESHOT: {
                FeedingMotror->SetTarget(FeedingMotror->GetTarget() + SINGLE_FEED_THETA);
                ShootMode = SHOOTILDE;
            } break;
            case DOUBLETAP: {
                FeedingMotror->SetTarget(FeedingMotror->GetTarget() + SINGLE_FEED_THETA);
                osDelay(200);
                FeedingMotror->SetTarget(FeedingMotror->GetTarget() + SINGLE_FEED_THETA);
                ShootMode = SHOOTILDE;
            } break;
            case RAPIDFIRE: {
                FeedingMotror->SetTarget(FeedingMotror->GetTarget() + 2 * SINGLE_FEED_THETA);
                ShootMode = SHOOTILDE;
            } break;
            default:
                break;
        }

        osDelay(1);
    }
}