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

ChassisRegConfig_t ChassisRegConfig;
communication::CanBridge* can_bridge = nullptr;
control::ChassisCanBridgeSender* chassis = nullptr;

// --------------------------- Chassis data code -------------------------

ChassisPID manual_theta_pid_init = {
    .kp = 400,
    .ki = 0.5,
    .kd = 20,
    .max_out = 500,
    .max_iout = 100,
};
ChassisData_t ChassisData;
control::ConstrainedPID* manual_mode_pid = nullptr;
float chassis_max[CVRAMPNUM] = {chassis_vx_max, chassis_vy_max, chassis_vz_max};
RampSource* v_ramp[CVRAMPNUM] = {nullptr};
BoolEdgeDetector* keyboard_edge[CKEY_EDGE_NUM] = {nullptr};
BoolEdgeDetector* ch_edge[CCH_EDGE_NUM] = {nullptr};
ChassisMotion_t ChassisMotion;

// ----------------------------- Chassis Main code -----------------------------

void InitChassis() {
    for (int i =0; i < CVRAMPNUM; i++) {
        v_ramp[i] = new RampSource(0, -chassis_max[i], chassis_max[i], 0.01);
    }

    for (int i =0; i < CKEY_EDGE_NUM; i++) {
        keyboard_edge[i] = new BoolEdgeDetector(false);
    }

    can_bridge = new communication::CanBridge(can1, ChassisRegConfig.tx_id);
    chassis = new control::ChassisCanBridgeSender(can_bridge, ChassisRegConfig.rx_id);
    chassis->SetChassisRegId(ChassisRegConfig.xy_reg_id, ChassisRegConfig.turn_on_reg_id,
        ChassisRegConfig.power_limit_reg_id, ChassisRegConfig.current_power_reg_id);
    chassis->Disable();
}

void ChassisRCUpdate() {

    if (RCIsOnline()) {
        rc_offline_start_time = 0;

        // 更新按键边沿检测
        if (rcData[CFORWARD] || rcData[CLEFT] || rcData[CBACKWARD] || rcData[CRIGHT]
            || rcData[CROTATELEFT] || rcData[CROTATERIGHT] || rcData[CSPEEDUP] || rcData[CSPEEDDOWN]) {
            keyboard_edge[CFORWARD_EDGE]->input(rcData[CFORWARD]);
            keyboard_edge[CLEFT_EDGE]->input(rcData[CLEFT]);
            keyboard_edge[CBACKWARD_EDGE]->input(rcData[CBACKWARD]);
            keyboard_edge[CRIGHT_EDGE]->input(rcData[CRIGHT]);
            keyboard_edge[CROTATE_LEFT_EDGE]->input(rcData[CROTATELEFT]);
            keyboard_edge[CROTATE_RIGHT_EDGE]->input(rcData[CROTATERIGHT]);
            keyboard_edge[CSPEED_UP_EDGE]->input(rcData[CSPEEDUP]);
            keyboard_edge[CSPEED_DOWN_EDGE]->input(rcData[CSPEEDDOWN]);
        } else {
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

            // 显式转换为布尔值
            ch_edge[CFB_EDGE]->input(rcData[CHASSIS_FB_CH] != 0);
            ch_edge[CLR_EDGE]->input(rcData[CHASSIS_LR_CH] != 0);
            ch_edge[CROTATE_EDGE]->input(rcData[CHASSIS_ROT_CH] != 0);
        }

        // 云台和底盘的角度差
        float relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);
        // 底盘以底盘自己为基准的运动速度
        ChassisMotion.sin_yaw = arm_sin_f32(relative_angle);
        ChassisMotion.cos_yaw = arm_cos_f32(relative_angle);

        // 前进速度控制（A/D 键）
        if (ch_edge[CFB_EDGE]->get()) {
            ChassisMotion.vx_set_org = rcData[CHASSIS_FB_CH];
        } else if (ch_edge[CFB_EDGE]->negEdge() || keyboard_edge[CSPEED_DOWN_EDGE]->posEdge()) {
            ChassisMotion.vx_set_org = 0;
            v_ramp[CVRAMP_X]->SetCurrent(0);
        } else if (keyboard_edge[CRIGHT_EDGE]->get()) {
            ChassisMotion.vx_set_org = v_ramp[CVRAMP_X]->Calc(ChassisData.current_speed_offset);
        } else if (keyboard_edge[CLEFT_EDGE]->get()) {
            ChassisMotion.vx_set_org = v_ramp[CVRAMP_X]->Calc(-ChassisData.current_speed_offset);
        } else {
            if (ChassisMotion.vx_set_org > 0) {
                ChassisMotion.vx_set_org = v_ramp[CVRAMP_X]->Calc(-ChassisData.current_speed_offset);
            } else if (ChassisMotion.vx_set_org < 0) {
                ChassisMotion.vx_set_org = v_ramp[CVRAMP_X]->Calc(ChassisData.current_speed_offset);
            }
        }

        // 平移速度控制（W/S 键）
        if (ch_edge[CLR_EDGE]->get()) {
            ChassisMotion.vy_set_org = rcData[CHASSIS_LR_CH];
        } else if (ch_edge[CLR_EDGE]->negEdge() || keyboard_edge[CSPEED_DOWN_EDGE]->posEdge()) {
            ChassisMotion.vy_set_org = 0;
            v_ramp[CVRAMP_Y]->SetCurrent(0);
        } else if (keyboard_edge[CFORWARD_EDGE]->get()) {
           ChassisMotion.vy_set_org = v_ramp[CVRAMP_Y]->Calc(ChassisData.current_speed_offset);
        } else if (keyboard_edge[CBACKWARD_EDGE]->get()) {
            ChassisMotion.vy_set_org = v_ramp[CVRAMP_Y]->Calc(-ChassisData.current_speed_offset);
        } else {
            if (ChassisMotion.vy_set_org > 0) {
                ChassisMotion.vy_set_org = v_ramp[CVRAMP_Y]->Calc(-ChassisData.current_speed_offset);
            } else if (ChassisMotion.vy_set_org < 0) {
                ChassisMotion.vy_set_org = v_ramp[CVRAMP_Y]->Calc(ChassisData.current_speed_offset);
            }
        }

        // 旋转速度控制（Q/E 键）
        if (ch_edge[CROTATE_EDGE]->get()) {
            ChassisMotion.offset_yaw = rcData[CHASSIS_ROT_CH];
        } else if (ch_edge[CROTATE_EDGE]->negEdge() || keyboard_edge[CSPEED_DOWN_EDGE]->posEdge()) {
            ChassisMotion.offset_yaw = 0;
            v_ramp[CVRAMP_Z]->SetCurrent(0);
        } else if (keyboard_edge[CROTATE_LEFT_EDGE]->get()) {
            ChassisMotion.offset_yaw = v_ramp[CVRAMP_Z]->Calc(ChassisData.current_speed_offset);
        } else if (keyboard_edge[CROTATE_RIGHT_EDGE]->get()) {
            ChassisMotion.offset_yaw = v_ramp[CVRAMP_Z]->Calc(-ChassisData.current_speed_offset);
        } else {
            if (ChassisMotion.offset_yaw > 0) {
                ChassisMotion.offset_yaw = v_ramp[CVRAMP_Z]->Calc(-ChassisData.current_speed_offset);
            } else if (ChassisMotion.offset_yaw < 0) {
                ChassisMotion.offset_yaw = v_ramp[CVRAMP_Z]->Calc(ChassisData.current_speed_offset);
            }
        }

        // 死区处理
        if (abs(ChassisMotion.vx_set_org) < 0.05f) {
            ChassisMotion.vx_set_org = 0;
        }
        if (abs(ChassisMotion.vy_set_org) < 0.05f) {
            ChassisMotion.vy_set_org = 0;
        }
        if (abs(ChassisMotion.offset_yaw) < 0.05f) {
            ChassisMotion.offset_yaw = 0;
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

void ChassisTask(void *arg) {
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

        if (SysStartData.GimbalInitFlag && ChassisMode == CMANUALMODE
            && RobotMode != RKILLMODE) {
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
            SysStartData.ShootInitFlag = 1;
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
                    RobotStatus.ChassisOnline = 0;
                    osDelay(10);
                    continue;  // 直接跳回循环开头
                }
                chassis->Disable();
                osDelay(10);
            }
        }

    if (ChassisMode != CKILLMODE) {
        chassis->Enable();
    }

        RobotStatus.ChassisOnline = 1;

        ChassisRCUpdate();

        ChassisMotion.chassis_vx = ChassisMotion.vx_set_org * ChassisData.ratio;
        ChassisMotion.chassis_vy = ChassisMotion.vy_set_org * ChassisData.ratio;
        ChassisMotion.chassis_vz = ChassisMotion.offset_yaw * ChassisData.ratio;
        ChassisMotion.vx_set = ChassisMotion.cos_yaw * ChassisMotion.chassis_vx +
            ChassisMotion.sin_yaw * ChassisMotion.chassis_vy;
        ChassisMotion.vy_set = -ChassisMotion.sin_yaw * ChassisMotion.chassis_vx +
            ChassisMotion.cos_yaw * ChassisMotion.chassis_vy;

        switch (ChassisMode) {
            case CKILLMODE:
                break;
            case CMANUALMODE: {
                ChassisMotion.vz_set = ChassisMotion.chassis_vz;
                chassis->SetSpeed(ChassisMotion.chassis_vx, ChassisMotion.chassis_vy,
                    ChassisMotion.vz_set);
            }break;
            case CFOLLOWMODE: {
                ChassisData.yaw_pid_error = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);
                ChassisData.manual_mode_pid_output = manual_mode_pid->ComputeOutput(ChassisData.yaw_pid_error);
                ChassisMotion.vz_set = ChassisData.manual_mode_pid_output * ChassisData.ratio;
                chassis->SetSpeed(ChassisMotion.vx_set, ChassisMotion.vy_set,
                    ChassisMotion.vz_set);

            }break;
            case CAUTOAIMODE: {
                ChassisMotion.vz_set = ChassisMotion.chassis_vz;
                chassis->SetSpeed(ChassisMotion.chassis_vx, ChassisMotion.chassis_vy,
                    ChassisMotion.vz_set);
            }break;
            case CSPINMODE: {
                if (ChassisMotion.offset_yaw != 0) {
                    ChassisData.spin_speed = ChassisData.spin_speed + ChassisData.offset_yaw;
                    ChassisData.offset_yaw = 0;
                    ChassisData.spin_speed = clip<float>(ChassisData.spin_speed, -660, 660);
                }
                ChassisMotion.vz_set = ChassisData.spin_speed * ChassisData.ratio;
                chassis->SetSpeed(ChassisMotion.vx_set, ChassisMotion.vy_set,
                    ChassisMotion.vz_set);
            }break;
            default:
                break;
        }

#ifdef HAS_REFEREE
    chassis->SetPower(true, referee->game_robot_status.chassis_power_limit,
                      referee->power_heat_data.chassis_power,
                      referee->power_heat_data.chassis_power_buffer);
#else
    chassis->SetPower(false, 30, 20, 60);
#endif

        osDelay(1);
    }

}