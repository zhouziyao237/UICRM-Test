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
    .yaw_offset_ = -1.9853f,         // 云台机械零点
    .pitch_max_ = 0.f,
    .yaw_max_ = PI,                  // ±PI度范围
    .yaw_circle_ = true,             // 能360度旋转
    .pitch_inverted = false,
    .yaw_inverted = true,
};

#define GimbalMinTheta -1.4309
#define FloatingRange 0.06
#define MinPositionStableTimeMs 500  // 最低点稳定时间阈值（毫秒）

driver::MotorCANBase* pitch_motor = nullptr;
driver::MotorCANBase* yaw_motor = nullptr;
control::Gimbal* gimbal = nullptr;
control::gimbal_data_t* gimbal_param = nullptr;
float pitch_diff, yaw_diff;
GimbalRatio_t GimbalRatio = {};
float actural_chassis_turn_speed = 0.0f;
static uint32_t min_position_start_time = 0;  // 记录进入最低点的时间

// 仅用于 follow / spin / autoaim 的“逻辑世界目标”
// 注意：这里存的是传给 TargetAbs 的逻辑目标，不是电机内部目标。
namespace {
    struct WorldTarget_t {
        float pitch;
        float yaw;
    };

    static WorldTarget_t gimbal_world_target = {0.0f, 0.0f};
    static uint8_t last_gimbal_mode = GKILLMODE;

    inline float WrapToPi(float angle) {
        while (angle > PI) angle -= 2.0f * PI;
        while (angle < -PI) angle += 2.0f * PI;
        return angle;
    }

    inline float LimitPitchCmd(float pitch_cmd) {
        return clip<float>(pitch_cmd, -gimbal_param->pitch_max_, gimbal_param->pitch_max_);
    }

    inline float LimitYawCmd(float yaw_cmd) {
        if (gimbal_param->yaw_circle_) {
            return WrapToPi(yaw_cmd);
        }
        return clip<float>(yaw_cmd, -gimbal_param->yaw_max_, gimbal_param->yaw_max_);
    }

    inline void SyncWorldTargetToCurrentPose() {
        // INS 是物理世界姿态；TargetAbs 的输入是“逻辑指令量”。
        // 因此这里要把当前物理姿态转成逻辑目标，才能保证切模式时目标连续。
        float pitch_cmd = INSTheta.pitch;
        float yaw_cmd = INSTheta.yaw;

        if (gimbal_param->pitch_inverted) pitch_cmd = -pitch_cmd;
        if (gimbal_param->yaw_inverted)   yaw_cmd   = -yaw_cmd;

        gimbal_world_target.pitch = LimitPitchCmd(pitch_cmd);
        gimbal_world_target.yaw   = LimitYawCmd(yaw_cmd);
    }

    inline void ApplyWorldDelta(float pitch_delta, float yaw_delta) {
        gimbal_world_target.pitch = LimitPitchCmd(gimbal_world_target.pitch + pitch_delta);
        gimbal_world_target.yaw   = LimitYawCmd(gimbal_world_target.yaw + yaw_delta);
    }

    inline void UpdateGimbalByWorldTarget() {
        gimbal->TargetAbs(gimbal_world_target.pitch, gimbal_world_target.yaw);
        gimbal->UpdateIMU(INSTheta.pitch, INSTheta.yaw);
    }
}

// ----------------------------- Gimbal Main code -----------------------------

void InitGimbal() {
    pitch_motor = new driver::Motor3508(can2, 0x204);
    yaw_motor = new driver::Motor6020(can1, 0x205);

    pitch_motor->SetTransmissionRatio(6);
    pitch_motor->ReInitPID(pitch_theta_pid_init, driver::MotorCANBase::ANGLE_LOOP_CONTROL);
    pitch_motor->ReInitPID(pitch_omega_pid_init, driver::MotorCANBase::SPEED_LOOP_CONTROL);
    pitch_motor->SetMode(driver::MotorCANBase::SPEED_LOOP_CONTROL |
                         driver::MotorCANBase::ANGLE_LOOP_CONTROL |
                         driver::MotorCANBase::ABSOLUTE);

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
    while (true) {
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
            rc_offline_start_time = 0;
            InitFlag = false;
            osDelay(10);
            continue;
        }

        if (GimbalMode == GMANUALMODE && RobotMode != RKILLMODE) {
            float pitch_theta = gimbal->getPitchByMotor();
            bool in_min_range = (pitch_theta <= GimbalMinTheta + FloatingRange) &&
                                (pitch_theta >= GimbalMinTheta - FloatingRange);
            if (in_min_range) {
                if (min_position_start_time == 0) {
                    min_position_start_time = bsp::GetHighresTickMilliSec();
                }

                uint32_t stable_time = bsp::GetHighresTickMilliSec() - min_position_start_time;
                if (stable_time >= MinPositionStableTimeMs) {
                    pitch_motor->align_angle_ = 0;
                    yaw_motor->Enable();
                    pitch_motor->Enable();
                } else {
                    osDelay(10);
                    continue;
                }
            } else {
                min_position_start_time = 0;
                is_robot_dead = true;
                osDelay(10);
                print("Gimbal is floating.\r\n");
                continue;
            }

            for (int i = 0; i < 5000; i++) {
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
            uint32_t cali_start_time = bsp::GetHighresTickMilliSec();
            const uint32_t CALI_TIMEOUT_MS = 10000;
            while (!ahrs->IsCailbrated()) {
                gimbal->TargetAbs(0, 0);
                gimbal->Update();
                osDelay(1);
                if (bsp::GetHighresTickMilliSec() - cali_start_time > CALI_TIMEOUT_MS) {
                    print("IMU Calibration Timeout!\r\n");
                    GimbalMode = GKILLMODE;
                    Buzzer_Sing(SingError);
                    break;
                }
            }

            if (!ahrs->IsCailbrated()) {
                continue;
            }

            Buzzer_Sing(SingCaliDone);
            print("Start Calibration.\r\n");
            SyncWorldTargetToCurrentPose();
            InitFlag = true;
        }

        if (InitFlag) {
            if (GimbalMode == GKILLMODE) {
                yaw_motor->Disable();
                pitch_motor->Disable();
                SysStartData.GimbalInitFlag = 0;
                rc_offline_start_time = 0;
                InitFlag = false;
                continue;
            }
            actural_chassis_turn_speed = chassis_vt / 6.0f;
            SysStartData.GimbalInitFlag = 1;
            last_gimbal_mode = GimbalMode;
            break;
        }
        osDelay(100);
    }

    while (true) {
        // 模式切换时只做目标同步，不改控制参考系含义以外的东西
        if (last_gimbal_mode != GimbalMode) {
            // 进入 follow / spin / autoaim 前，把世界目标同步到当前姿态，避免跳大角
            if ((GimbalMode == GFOLLOWMODE) || (GimbalMode == GSPINMODE) ||
                (GimbalMode == GAUTOAIMODE)) {
                SyncWorldTargetToCurrentPose();
            }
            last_gimbal_mode = GimbalMode;
        }

        if (GimbalMode == GKILLMODE) {
            pitch_motor->Disable();
            yaw_motor->Disable();
            while (GimbalMode == GKILLMODE) {
                osDelay(10);
            }
            pitch_motor->Enable();
            yaw_motor->Enable();
            SyncWorldTargetToCurrentPose();
            last_gimbal_mode = GimbalMode;
            continue;
        }

        if (GimbalMode != GKILLMODE) {
            pitch_motor->Enable();
            yaw_motor->Enable();
        }

        if (!pitch_motor->IsOnline() || !yaw_motor->IsOnline()) {
            pitch_motor->Disable();
            yaw_motor->Disable();
            RobotStatus.GimbalOnline = 0;
            continue;
        } else {
            pitch_motor->Enable();
            yaw_motor->Enable();
        }
        RobotStatus.GimbalOnline = 1;

        GimbalRCUpdata();

        float pitch_target = clip<float>(GimbalRatio.pitch_ratio, -gimbal_param->pitch_max_,
                                         gimbal_param->pitch_max_);
        float yaw_target = wrap<float>(GimbalRatio.yaw_ratio, -gimbal_param->yaw_max_,
                                       gimbal_param->yaw_max_);

        pitch_diff = clip<float>(pitch_target, -PI, PI);
        yaw_diff = WrapToPi(yaw_target);

        const float ratio = 0.1875f;
        actural_chassis_turn_speed = chassis_vt * ratio * 0.25f;
        yaw_motor->SetSpeedOffset(actural_chassis_turn_speed);

        switch (GimbalMode) {
            case GMANUALMODE: {
                // 手动：保持你原来的局部控制逻辑
                gimbal->TargetRel(pitch_diff, yaw_diff);
                gimbal->Update();
                break;
            }
            case GFOLLOWMODE: {
                // 跟随：云台仍由用户控制，但目标放在世界坐标；底盘在 chassis_task 里去跟随云台
                ApplyWorldDelta(pitch_diff, yaw_diff);
                UpdateGimbalByWorldTarget();
                break;
            }
            case GAUTOAIMODE: {
                // 若后续接自瞄，直接写 gimbal_world_target 再 UpdateGimbalByWorldTarget() 即可
                UpdateGimbalByWorldTarget();
                break;
            }
            case GSPINMODE: {
                // 小陀螺：底盘转，但云台继续保持用户手感一致（世界坐标控制）
                ApplyWorldDelta(pitch_diff, yaw_diff);
                UpdateGimbalByWorldTarget();
                break;
            }
            default:
                break;
        }

        osDelay(10);
    }
}
