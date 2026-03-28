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

#include "gimbal_task.h"

// ----------------------------- Gimbal Task code -----------------------------

osThreadId_t GimbalTaskHandle;

// ----------------------------- Gimbal Config code -----------------------------

// 配置你的云台（±60度，1:6减速比）
static control::gimbal_data_t gimbal_config = {
    .pitch_offset_ = 0.9371f,        // 云台机械零点
    .yaw_offset_ = -1.9853f,           // 云台机械零点
    .pitch_max_ = 0.f,
    .yaw_max_ = PI,            // ±PI度范围
    .yaw_circle_ = true,          // 能360度旋转
    .pitch_inverted = false,        // 不反转
    .yaw_inverted = true,         // 不反转
    .pitch_reduction = 6.0f,   // 1:6减速比
    .yaw_reduction = 1.0f          // 1:6减速比
};

#define GimbalMinTheta -1.4253
#define FloatingRange 0.03
#define MinPositionStableTimeMs 500  // 最低点稳定时间阈值（毫秒）

driver::MotorCANBase* pitch_motor = nullptr;
driver::MotorCANBase* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::gimbal_data_t* gimbal_param = nullptr;
float pitch_diff, yaw_diff;
GimbalRatio_t GimbalRatio = {};
float actural_chassis_turn_speed = 0.0f;
static uint32_t min_position_start_time = 0;  // 记录进入最低点的时间

// ----------------------------- Gimbal Main code -----------------------------

void InitGimbal() {
    pitch_motor = new driver::Motor3508(can2, 0x204);
    yaw_motor = new driver::Motor6020(can1, 0x205);

    pitch_motor->SetTransmissionRatio(1);
    pitch_motor->ReInitPID(pitch_theta_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    pitch_motor->ReInitPID(pitch_omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    pitch_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                         driver::MotorCANBase::ANGLE_LOOP_CONTROL |
                         driver::MotorCANBase::SINGLETURN);

    yaw_motor->SetTransmissionRatio(1);
    yaw_motor->ReInitPID(yaw_theta_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
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

void GimbalRCUpdata() {
    if (RCIsOnline()) {
        rc_offline_start_time = 0;

        if (rcData[GIMBAL_MOUSE_Y_CH] != 0 || rcData[GIMBAL_MOUSE_X_CH] != 0) {
            if (MOUSE_PitchInverted) {
                rcData[GIMBAL_MOUSE_Y_CH] = -rcData[GIMBAL_MOUSE_Y_CH];
            }
            if (MOUSE_YawInverted) {
                rcData[GIMBAL_MOUSE_X_CH] = -rcData[GIMBAL_MOUSE_X_CH];
            }
            GimbalRatio.pitch_ratio = rcData[GIMBAL_MOUSE_Y_CH] / mouse_xy_max * mouse_ratio;
            GimbalRatio.yaw_ratio = rcData[GIMBAL_MOUSE_X_CH] / mouse_xy_max * mouse_ratio;
        } else {
            if (CH_PitchInverted) {
                rcData[GIMBAL_PITCH_CH] = -rcData[GIMBAL_PITCH_CH];
            }
            if (CH_YawInverted) {
                rcData[GIMBAL_YAW_CH] = -rcData[GIMBAL_YAW_CH];
            }
            GimbalRatio.pitch_ratio = rcData[GIMBAL_PITCH_CH] / MAXCHECK * remote_ratio;
            GimbalRatio.yaw_ratio = rcData[GIMBAL_YAW_CH] / MAXCHECK * remote_ratio;
        }
    } else {
        // 离线计时
        if (rc_offline_start_time == 0) {
            rc_offline_start_time = bsp::GetHighresTickMilliSec();
        }

        // 双重确认：VTM(100ms) + 应用层 (200ms) = 总共 300ms 才触发急停
        if (bsp::GetHighresTickMilliSec() - rc_offline_start_time > APP_OFFLINE_DEBOUNCE_MS) {
            GimbalMode = GKILLMODE;
        }
        // 否则保持当前模式
    }
}

uint8_t GimbalIsOnline() {
    return RobotStatus.GimbalOnline;
}

void GimbalTask(void* arg) {
    UNUSED(arg);

#ifdef HAS_REFEREE
    if (RefereeIsOnline()) {
        while (!Referee_data->game_robot_status.mains_power_gimbal_output) {
            RobotMode = RKILLMODE;
            GimbalMode = GKILLMODE;
            osDelay(10);
        }
    }
#endif

    if (!gimbal || !pitch_motor || !yaw_motor)
        RM_ASSERT_TRUE(false, "Gimbal Init Failed!");

    yaw_motor->Disable();
    pitch_motor->Disable();

    bool InitFlag = false;
    while (true){

        // todo: 暂时测试头时yaw没有所有没加  “!yaw_motor->IsOnline()”
        if (!pitch_motor->IsOnline() || !yaw_motor->IsOnline()) {
            osDelay(10);
            GimbalMode = GKILLMODE;
            RobotStatus.GimbalOnline = 0;
            InitFlag = false;
        }
        RobotStatus.GimbalOnline = 1;

        if (GimbalMode == GKILLMODE) {
            yaw_motor->Disable();
            pitch_motor->Disable();
            InitFlag = false;
            osDelay(10);
            continue;
        }

        if (GimbalMode == GMANUALMODE && RobotMode != RKILLMODE) {

            float pitch_theta = gimbal->getPitchByMotor();
            bool in_min_range = (pitch_theta <= GimbalMinTheta + FloatingRange) &&
                                (pitch_theta >= GimbalMinTheta - FloatingRange);
            if (in_min_range) {
                // 在最低点范围内，检查稳定时间
                if (min_position_start_time == 0) {
                    min_position_start_time = bsp::GetHighresTickMilliSec();
                }

                uint32_t stable_time = bsp::GetHighresTickMilliSec() - min_position_start_time;
                if (stable_time >= MinPositionStableTimeMs) {
                    // 稳定在最低点足够长时间，开始初始化
                    pitch_motor->align_angle_ = 0;

            yaw_motor->Enable();
            pitch_motor->Enable();
                } else {
                    // 等待稳定
                osDelay(10);
                    continue;
                }
            } else {
                // 不在最低点范围内，重置计时器
                min_position_start_time = 0;
                is_robot_dead = true;
                osDelay(10);
                print("Gimbal is floating.\r\n");
                continue;
            }

            int i = 0;
            for (i = 0; i < 5000; i++) {
                gimbal->TargetAbs(0, 0);
                gimbal->Update();
                osDelay(1);
            }

            if (GimbalMode == GKILLMODE) {
                yaw_motor->Disable();
                pitch_motor->Disable();
                InitFlag = false;
                continue;
            }

            Buzzer_Sing(SingCaliStart);
            ahrs->Cailbrate();
            i = 0;
            while (!ahrs->IsCailbrated()) {
                gimbal->TargetAbs(0, 0);
                gimbal->Update();
                osDelay(1);
                ++i;
            }

            Buzzer_Sing(SingCaliDone);
            print("Start Calibration.\r\n");
            InitFlag = true;
        }

        if (InitFlag) {
            if (GimbalMode == GKILLMODE) {
                yaw_motor->Disable();
                pitch_motor->Disable();
                SysStartData.GimbalInitFlag = 0;
                InitFlag = false;
                continue;
            }
            actural_chassis_turn_speed = chassis_vt / 6.0f;
            SysStartData.GimbalInitFlag = 1;
            break;
        }
        osDelay(100);
    }

    while (true) {
        if (GimbalMode == GKILLMODE) {
            while (true) {
                if (GimbalMode != GKILLMODE) {
                    pitch_motor->Enable();
                    yaw_motor->Enable();
                    break;
                }
                pitch_motor->Disable();
                yaw_motor->Disable();
                osDelay(10);
            }
        }

        if (GimbalMode != GKILLMODE) {
            pitch_motor->Enable();
            yaw_motor->Enable();
        }

        // todo: 暂时测试头时yaw没有所有没加  “!yaw_motor->IsOnline()”
        if (!pitch_motor->IsOnline() || !yaw_motor->IsOnline()) {
            GimbalMode = GKILLMODE;
            pitch_motor->Disable();
            yaw_motor->Disable();
            RobotStatus.GimbalOnline = 0;
            continue;
        }
        RobotStatus.GimbalOnline = 1;

        GimbalRCUpdata();

        float pitch_target = clip<float>(GimbalRatio.pitch_ratio, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        float yaw_target = wrap<float>(GimbalRatio.yaw_ratio, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);

        pitch_diff = clip<float>(pitch_target, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
        yaw_diff = wrap<float>(yaw_target, -PI, PI);
        // 根据运动模式选择不同的控制方式
        float yaw_speed_offset = actural_chassis_turn_speed + GimbalRatio.yaw_ratio;
        float pitch_speed_offset = GimbalRatio.pitch_ratio;
        yaw_motor->SetSpeedOffset(yaw_speed_offset);
        pitch_motor->SetSpeedOffset(pitch_speed_offset);

        switch (GimbalMode) {
            case GMANUALMODE:{
                // 如果是高级模式，将电机获取的云台当前角度作为参考系
                gimbal->TargetRel(pitch_diff, yaw_diff);
                gimbal->Update();
                break;
            }
            case GFOLLOWMODE:
                // 如果是跟随模式或者旋转模式，将IMU作为参考系
                gimbal->TargetRel(pitch_diff, yaw_diff);
                gimbal->UpdateIMU(INSTheta.pitch, INSTheta.yaw);
                break;
            case GAUTOAIMODE: {
                // gimbal->TargetAbs(minipc->target_angle.target_pitch,
                //                   minipc->target_angle.target_yaw);
                // gimbal->UpdateIMU(INSTheta.pitch, INSTheta.yaw);
                break;
            }
            case GSPINMODE:
                gimbal->TargetRel(pitch_diff, yaw_diff);
                gimbal->UpdateIMU(INSTheta.pitch, INSTheta.yaw);
                break;

            default:
                break;
        }

        osDelay(10);
    }
}

