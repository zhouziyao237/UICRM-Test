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
// Created by Administrator on 2026/3/3.
//

#include "chassis_task.h"

// ----------------------------- Chassis Task code -----------------------------

osThreadId_t ChassisTaskHandle;
// --------------------------- Chassis CanBridge data code -------------------------

ChassisRegConfig_t ChassisRegConfig = {.tx_id = 0x51,
                                       .rx_id = 0x52,
                                       .xy_reg_id = 0x70,
                                       .turn_on_reg_id = 0x71,
                                       .power_limit_reg_id = 0x72,
                                       .current_power_reg_id = 0x73};
communication::CanBridge* can_bridge = nullptr;
control::ChassisCanBridgeSender* chassis = nullptr;

// --------------------------- Chassis data code -------------------------

ChassisPID manual_theta_pid_init = {
    .kp = 200,
    .ki = 0,
    .kd = 1600,
    .max_out = 500,
    .max_iout = 100,
};
ChassisData_t ChassisData;
control::ConstrainedPID* manual_mode_pid = nullptr;
float chassis_max[CVRAMPNUM] = {chassis_vx_max, chassis_vy_max, chassis_vz_max};
RampSource* v_ramp[CVRAMPNUM] = {nullptr};
BoolEdgeDetector* keyboard_edge[CKEY_EDGE_NUM] = {nullptr};
bsp::BatteryVol* battery_vol = nullptr;
ChassisMotion_t ChassisMotion;

// ----------------------------- Chassis Main code -----------------------------

void InitChassis() {
    for (int i = 0; i < CVRAMPNUM; i++) {
        v_ramp[i] = new RampSource(0, -chassis_max[i], chassis_max[i], 0.01);
    }

    for (int i = 0; i < CKEY_EDGE_NUM; i++) {
        keyboard_edge[i] = new BoolEdgeDetector(false);
    }

    manual_mode_pid = new control::ConstrainedPID(
        manual_theta_pid_init.kp, manual_theta_pid_init.ki, manual_theta_pid_init.kd,
        manual_theta_pid_init.max_iout, manual_theta_pid_init.max_out);

    battery_vol = new bsp::BatteryVol(&hadc3, ADC_CHANNEL_8, 1, ADC_SAMPLETIME_3CYCLES);

    can_bridge = new communication::CanBridge(can1, ChassisRegConfig.tx_id);
    chassis = new control::ChassisCanBridgeSender(can_bridge, ChassisRegConfig.rx_id);
    chassis->SetChassisRegId(ChassisRegConfig.xy_reg_id, ChassisRegConfig.turn_on_reg_id,
                             ChassisRegConfig.power_limit_reg_id,
                             ChassisRegConfig.current_power_reg_id);
    chassis->Disable();
}

void ChassisRCUpdate() {
    if (RCIsOnline()) {
        rc_offline_start_time = 0;

        // 更新按键边沿检测
        if (rcData[CHASSIS_FB_CH] || rcData[CHASSIS_LR_CH] || rcData[CHASSIS_ROT_CH]) {

            // 摇杆模式（带反向）
            if (CHASSIS_FB_REV) {
                rcData[CHASSIS_FB_CH] = -rcData[CHASSIS_FB_CH];
            }
            if (CHASSIS_LR_REV) {
                rcData[CHASSIS_LR_CH] = -rcData[CHASSIS_LR_CH];
            }
            if (CHASSIS_ROT_REV) {
                rcData[CHASSIS_ROT_CH] = -rcData[CHASSIS_ROT_CH];
            }

                ChassisMotion.vx_set_org = (float)rcData[CHASSIS_FB_CH] / 660;
                ChassisMotion.vy_set_org = (float)rcData[CHASSIS_LR_CH] / 660;
                // ChassisMotion.offset_yaw = (float)rcData[CHASSIS_ROT_CH] / 660;


        } else if (rcData[CFORWARD] || rcData[CLEFT] || rcData[CBACKWARD] || rcData[CRIGHT] ||
            rcData[CROTATELEFT] || rcData[CROTATERIGHT] || rcData[CSPEEDUP] || rcData[CSPEEDDOWN]){

            // keyboard_edge[CFORWARD_EDGE]->input(rcData[CFORWARD]);
            // keyboard_edge[CLEFT_EDGE]->input(rcData[CLEFT]);
            // keyboard_edge[CBACKWARD_EDGE]->input(rcData[CBACKWARD]);
            // keyboard_edge[CRIGHT_EDGE]->input(rcData[CRIGHT]);
            // keyboard_edge[CROTATE_LEFT_EDGE]->input(rcData[CROTATELEFT]);
            // keyboard_edge[CROTATE_RIGHT_EDGE]->input(rcData[CROTATERIGHT]);
            // keyboard_edge[CSPEED_UP_EDGE]->input(rcData[CSPEEDUP]);
            // keyboard_edge[CSPEED_DOWN_EDGE]->input(rcData[CSPEEDDOWN]);
            //
            // const float keyboard_speed = keyboard_edge[CSPEED_UP_EDGE] ? 1 : 0.5;
            // const float keyboard_spin_speed = 1;
            // ChassisMotion.vx_set_org = (keyboard_edge[CRIGHT_EDGE] - keyboard_edge[CLEFT_EDGE]) * keyboard_speed;
            // ChassisMotion.vy_set_org = (keyboard_edge[CFORWARD_EDGE] - keyboard_edge[CBACKWARD_EDGE]) * keyboard_speed;
            // ChassisMotion.offset_yaw = (keyboard_edge[CROTATE_RIGHT_EDGE] - keyboard_edge[CROTATE_LEFT_EDGE]) * keyboard_spin_speed;

        }


    } else {
        if (rc_offline_start_time == 0) {
            rc_offline_start_time = bsp::GetHighresTickMilliSec();
        }

        // 双重确认：VTM(100ms) + 应用层 (200ms) = 总共 300ms 才触发急停
        if (bsp::GetHighresTickMilliSec() - rc_offline_start_time > APP_OFFLINE_DEBOUNCE_MS) {
            ChassisMode = CKILLMODE;
        }
    }
}

uint8_t ChassisOnline() {
    return RobotStatus.ChassisOnline;
}

void Power_Ctrl() {
#ifdef HAS_REFEREE
    chassis->UpdatePower(true,
        Referee_data->game_robot_status.chassis_power_limit - 5,
        battery_vol->GetBatteryVol(),
        Referee_data->power_heat_data.chassis_power_buffer);

#else
    chassis->UpdatePower(true, 60, battery_vol->GetBatteryVol(), 60);
#endif
}

void ChassisTask(void* arg) {
    UNUSED(arg);

    bool InitFlag = false;
    RobotStatus.ChassisOnline = 1;

    chassis->Disable();

    while (true) {
        if (ChassisMode == CKILLMODE) {
            chassis->Disable();
            InitFlag = false;
            osDelay(10);
            continue;
        }

        if (SysStartData.GimbalInitFlag && ChassisMode == CMANUALMODE && RobotMode != RKILLMODE) {
            InitFlag = true;
            osDelay(10);
        }

        if (InitFlag) {
            if (ChassisMode == CKILLMODE) {
                chassis->Disable();
                SysStartData.ChassisInitFlag = 0;
                InitFlag = false;
                continue;
            }
            SysStartData.ChassisInitFlag = 1;
            manual_mode_pid->Reset();
            chassis->Enable();
            break;
        }
        osDelay(100);
    }

    while (true) {
        if (ChassisMode == CKILLMODE) {
            while (true) {
                if (ChassisMode != CKILLMODE) {
                    chassis->Enable();
                    // 清除所有 Ramp 状态
                    for (int i = 0; i < CVRAMPNUM; i++) {
                        v_ramp[i]->SetCurrent(0);
                    }
                    ChassisMotion.vx_set_org = 0;
                    ChassisMotion.vy_set_org = 0;
                    ChassisMotion.offset_yaw = 0;
                    RobotStatus.ChassisOnline = 1;
                    chassis->Enable();
                    osDelay(10);
                    break;
                }
                chassis->Disable();
                RobotStatus.ChassisOnline = 0;
                osDelay(10);
            }
        }

        if (ChassisMode != CKILLMODE) {
            chassis->Enable();
        }

        RobotStatus.ChassisOnline = 1;

        ChassisRCUpdate();

        // 云台和底盘的角度差
        ChassisMotion.chassis_yaw_diff = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

        // 底盘以底盘自己为基准的运动速度
        ChassisMotion.sin_yaw = arm_sin_f32(ChassisMotion.chassis_yaw_diff);
        ChassisMotion.cos_yaw = arm_cos_f32(ChassisMotion.chassis_yaw_diff);
        ChassisMotion.chassis_vx = ChassisMotion.cos_yaw * ChassisMotion.vx_set_org + ChassisMotion.sin_yaw * ChassisMotion.vy_set_org;
        ChassisMotion.chassis_vy = -ChassisMotion.sin_yaw * ChassisMotion.vx_set_org + ChassisMotion.cos_yaw * ChassisMotion.vy_set_org;
        ChassisMotion.chassis_vz = 0;


        switch (ChassisMode) {
            case CKILLMODE:
                break;
            case CMANUALMODE: {
                ChassisMotion.chassis_vx = ChassisMotion.vx_set_org;
                ChassisMotion.chassis_vy = ChassisMotion.vy_set_org;
                ChassisMotion.chassis_vz = ChassisMotion.offset_yaw;
            } break;
            case CFOLLOWMODE: {
                // 读取底盘和云台yaw轴角度差，控制底盘转向云台的方向
                const float angle_threshold = 0.02f;
                ChassisData.yaw_pid_error = ChassisMotion.chassis_yaw_diff;
                if (fabs(ChassisData.yaw_pid_error) < angle_threshold) {
                    ChassisData.yaw_pid_error = 0;
                }

                ChassisData.chassis_vt_pid = manual_mode_pid->ComputeOutput(ChassisData.yaw_pid_error);
                if (ChassisData.yaw_pid_error != 0)
                    ChassisMotion.chassis_vz = ChassisData.chassis_vt_pid;

            } break;
            case CAUTOAIMODE: {
                ChassisMotion.chassis_vx = ChassisMotion.vx_set_org;
                ChassisMotion.chassis_vy = ChassisMotion.vy_set_org;
                ChassisMotion.chassis_vz = ChassisMotion.offset_yaw;
            } break;
            case CSPINMODE: {

                // 小陀螺模式，拨盘用来控制底盘加速度
                static float spin_speed = 0.5;
                spin_speed = spin_speed + ChassisMotion.offset_yaw * 0.01;
                spin_speed = clip<float>(spin_speed, -1, 1);
                ChassisMotion.chassis_vz = spin_speed;

            } break;
            default:
                break;
        }

        // 进行缩放
        ChassisMotion.chassis_vx *= ChassisData.chassis_max_xy_speed;
        ChassisMotion.chassis_vy *= ChassisData.chassis_max_xy_speed;
        ChassisMotion.offset_yaw *= ChassisData.chassis_max_t_speed;

        static const float move_ease_ratio = 1.8;
        static const float turn_ease_ratio = 0.9;
        static Ease chassis_ease_vx(0, move_ease_ratio);
        static Ease chassis_ease_vy(0, move_ease_ratio);
        static Ease chassis_ease_vt(0, turn_ease_ratio);
        ChassisMotion.chassis_vx = chassis_ease_vx.Calc(ChassisMotion.chassis_vx);
        ChassisMotion.chassis_vy = chassis_ease_vy.Calc(ChassisMotion.chassis_vy);
        ChassisMotion.offset_yaw = chassis_ease_vt.Calc(ChassisMotion.offset_yaw);

        Power_Ctrl();
        osDelay(5);
        chassis->SetSpeed(ChassisMotion.chassis_vx, ChassisMotion.chassis_vy,
                          ChassisMotion.chassis_vz);
        osDelay(5);
    }
}