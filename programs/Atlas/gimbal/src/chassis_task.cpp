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

#include "chassis_task.h"
osThreadId_t chassisTaskHandle;

const float chassis_max_xy_speed = 2 * PI;
const float chassis_max_t_speed = 4 * PI;

float sin_yaw, cos_yaw, vx_set = 0, vy_set = 0, vz_set = 0, vx_set_org = 0, vy_set_org = 0;
float chassis_vx = 0, chassis_vy = 0, chassis_vz = 0;

float offset_yaw = 0;
float spin_speed = 350;
float manual_mode_yaw_pid_args[3] = {400, 0.5, 20};
float manual_mode_yaw_pid_max_iout = 100;
float manual_mode_yaw_pid_max_out = 500;
control::ConstrainedPID* manual_mode_pid = new control::ConstrainedPID(
    manual_mode_yaw_pid_args, manual_mode_yaw_pid_max_iout, manual_mode_yaw_pid_max_out);
float yaw_pid_error = 0;
float manual_mode_pid_output = 0;

bool chassis_boost_flag = true;
float ratio = 1.0f / 660.0f * 10 * PI;

# define speed_offset 660
# define speed_offset_boost 1320
float current_speed_offset = speed_offset;
remote::keyboard_t keyboard;
remote::keyboard_t last_keyboard;
RampSource* vx_ramp = new RampSource(0, -chassis_vx_max / 2, chassis_vx_max / 2, 0.01);
RampSource* vy_ramp = new RampSource(0, -chassis_vy_max / 2, chassis_vy_max / 2, 0.01);
RampSource* vz_ramp = new RampSource(0, -chassis_vz_max / 2, chassis_vz_max / 2, 0.01);

BoolEdgeDetector* w_edge = new BoolEdgeDetector(false);
BoolEdgeDetector* s_edge = new BoolEdgeDetector(false);
BoolEdgeDetector* a_edge = new BoolEdgeDetector(false);
BoolEdgeDetector* d_edge = new BoolEdgeDetector(false);
BoolEdgeDetector* ctrl_edge = new BoolEdgeDetector(false);

BoolEdgeDetector* q_edge = new BoolEdgeDetector(false);
BoolEdgeDetector* e_edge = new BoolEdgeDetector(false);
BoolEdgeDetector* x_edge = new BoolEdgeDetector(false);

BoolEdgeDetector* ch0_edge = new BoolEdgeDetector(false);
BoolEdgeDetector* ch1_edge = new BoolEdgeDetector(false);
BoolEdgeDetector* ch2_edge = new BoolEdgeDetector(false);
BoolEdgeDetector* ch3_edge = new BoolEdgeDetector(false);
BoolEdgeDetector* ch4_edge = new BoolEdgeDetector(false);

communication::CanBridge* can_bridge = nullptr;
control::ChassisCanBridgeSender* chassis = nullptr;

void ADVANCEDMODE() {
    vz_set = chassis_vz;

    chassis->SetSpeed(chassis_vx, chassis_vy, vz_set);
    osDelay(1);
#ifdef HAS_REFEREE
    chassis->SetPower(true, referee->game_robot_status.chassis_power_limit,
                      referee->power_heat_data.chassis_power,
                      referee->power_heat_data.chassis_power_buffer);
    osDelay(1);
#else
    chassis->SetPower(false, 30, 20, 60);
#endif
}

void AUTOAIMMODE() {
    vz_set = chassis_vz;

    chassis->SetSpeed(chassis_vx, chassis_vy, vz_set);
    osDelay(1);
#ifdef HAS_REFEREE
    chassis->SetPower(true, referee->game_robot_status.chassis_power_limit,
                      referee->power_heat_data.chassis_power,
                      referee->power_heat_data.chassis_power_buffer);
    osDelay(1);
#else
    chassis->SetPower(false, 30, 20, 60);
#endif
}

void FOLLOWMODE() {
    // 读取底盘和云台yaw轴角度差，控制底盘转向云台的方向
    yaw_pid_error = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);

    manual_mode_pid_output = manual_mode_pid->ComputeOutput(yaw_pid_error);
    vz_set = manual_mode_pid_output * ratio;
    chassis->SetSpeed(vx_set, vy_set, vz_set);
    osDelay(1);
#ifdef HAS_REFEREE
    chassis->SetPower(true, referee->game_robot_status.chassis_power_limit,
                      referee->power_heat_data.chassis_power,
                      referee->power_heat_data.chassis_power_buffer);
#else
    chassis->SetPower(false, 30, 20, 60);
#endif
    osDelay(1);
}

void SPINMODE() {
    // 小陀螺模式，拨盘用来控制底盘加速度
    if (offset_yaw != 0) {
        spin_speed = spin_speed + offset_yaw;
        offset_yaw = 0;
        spin_speed = clip<float>(spin_speed, -660, 660);
    }
    vz_set = spin_speed * ratio;
    chassis->SetSpeed(vx_set, vy_set, vz_set);
    osDelay(1);
#ifdef HAS_REFEREE
    chassis->SetPower(true, referee->game_robot_status.chassis_power_limit,
                      referee->power_heat_data.chassis_power,
                      referee->power_heat_data.chassis_power_buffer);
#else
chassis->SetPower(false, 30, 20, 60);
#endif
    osDelay(1);
}

void MOVEMENTMODE() {
    vx_set = chassis_vx * ratio;
    vy_set = chassis_vy * ratio;
    vz_set = chassis_vz * ratio;

    chassis->SetSpeed(vx_set, vy_set, vz_set);
    osDelay(1);
#ifdef HAS_REFEREE
    chassis->SetPower(true, referee->game_robot_status.chassis_power_limit,
                      referee->power_heat_data.chassis_power,
                      referee->power_heat_data.chassis_power_buffer);
#else
chassis->SetPower(false, 30, 20, 60);
#endif
    osDelay(1);
}

void chassisTask(void* arg) {
    UNUSED(arg);
    kill_chassis();
    osDelay(500);

    while(remote_mode == REMOTE_MODE_KILL || protect_wraning_flag) {
        kill_chassis();
        osDelay(CHASSIS_OS_DELAY);
    }

    // 等待IMU初始化
    while (!ahrs->IsCailbrated()) {
        osDelay(1);
    }

    manual_mode_pid->Reset();
    chassis->Enable();


    while (true) {

#ifdef HAS_REFEREE
        while (remote_mode == REMOTE_MODE_KILL || !referee->game_robot_status.mains_power_chassis_output) {
#else
        while (remote_mode == REMOTE_MODE_KILL) {
#endif
            kill_chassis();
            osDelay(CHASSIS_OS_DELAY + 2);
        }

        chassis->Enable();

        // todo: 遥控器与在线键盘控制底盘速度逻辑
        chassis_remote_mode();

        chassis_vx = vx_set_org * ratio;
        chassis_vy = vy_set_org * ratio;
        chassis_vz = offset_yaw * ratio;
        vx_set = cos_yaw * chassis_vx + sin_yaw * chassis_vy;
        vy_set = -sin_yaw * chassis_vx + cos_yaw * chassis_vy;

        switch (remote_mode) {
            case REMOTE_MODE_ADVANCED:
                ADVANCEDMODE();
                break;

            case REMOTE_MODE_AUTOAIM:
                AUTOAIMMODE();
                break;

            case REMOTE_MODE_FOLLOW:
                // 读取底盘和云台yaw轴角度差，控制底盘转向云台的方向
                FOLLOWMODE();
                break;

            case REMOTE_MODE_SPIN:
                // 小陀螺模式，拨盘用来控制底盘加速度
                SPINMODE();
                break;

            case REMOTE_MODE_PREPARE_HAND_MOVEMENT:
                // 手动模式下，遥控器直接控制底盘速度
                MOVEMENTMODE();
                break;

            default:
                break;
        }

        chassis_vz = vz_set;
        osDelay(CHASSIS_OS_DELAY);
    }
}

void chassis_remote_mode() {
#ifdef DBUS_MODE
    // 更新状态机
    {
        last_keyboard = keyboard;
        if (dbus->IsOnline()) {
            keyboard = dbus->keyboard;
            ch0_edge->input(dbus->ch0 != 0);
            ch1_edge->input(dbus->ch1 != 0);
            ch2_edge->input(dbus->ch2 != 0);
            ch3_edge->input(dbus->ch3 != 0);
            ch4_edge->input(dbus->ch4 != 0);
        } else if (refereerc->IsOnline()) {
            keyboard = refereerc->remote_control.keyboard;
            ch0_edge->input(false);
            ch1_edge->input(false);
            ch2_edge->input(false);
            ch3_edge->input(false);
            ch4_edge->input(false);
        }
    }
    {
        w_edge->input(keyboard.bit.W);
        s_edge->input(keyboard.bit.S);
        a_edge->input(keyboard.bit.A);
        d_edge->input(keyboard.bit.D);
        ctrl_edge->input(keyboard.bit.CTRL);

        q_edge->input(keyboard.bit.Q);
        e_edge->input(keyboard.bit.E);
        x_edge->input(keyboard.bit.X);
    }

    // 云台和底盘的角度差
    float relative_angle = yaw_motor->GetThetaDelta(gimbal_param->yaw_offset_);
    // 底盘以底盘自己为基准的运动速度
    sin_yaw = arm_sin_f32(relative_angle);
    cos_yaw = arm_cos_f32(relative_angle);

    // 前进速度控制
    if (ch1_edge->get()) {
        vx_set_org = dbus->ch1;
    } else if (ch1_edge->negEdge() || x_edge->posEdge()) {
        vx_set_org = 0;
        vx_ramp->SetCurrent(0);
    } else if (d_edge->get()) {
        vx_set_org = vx_ramp->Calc(current_speed_offset);
    } else if (a_edge->get()) {
        vx_set_org = vx_ramp->Calc(-current_speed_offset);
    } else {
        if (vx_set_org > 0) {
            vx_set_org = vx_ramp->Calc(-current_speed_offset);
        } else if (vx_set_org < 0) {
            vx_set_org = vx_ramp->Calc(current_speed_offset);
        }
    }
    // 平移速度控制
    if (ch0_edge->get()) {
        // todo: 摇杆反向
        vy_set_org = -dbus->ch0;
    } else if (ch0_edge->negEdge() || x_edge->posEdge()) {
        vy_set_org = 0;
        vy_ramp->SetCurrent(0);
    } else if (w_edge->get()) {
        vy_set_org = vy_ramp->Calc(current_speed_offset);
    } else if (s_edge->get()) {
        vy_set_org = vy_ramp->Calc(-current_speed_offset);
    } else {
        if (vy_set_org > 0) {
            vy_set_org = vy_ramp->Calc(-current_speed_offset);
        } else if (vy_set_org < 0) {
            vy_set_org = vy_ramp->Calc(current_speed_offset);
        }
    }
    // 旋转速度控制（如果需要）
    if (ch4_edge->get()) {
        offset_yaw = dbus->ch4;
    } else if (ch4_edge->negEdge() || x_edge->posEdge()) {
        offset_yaw = 0;
        vz_ramp->SetCurrent(0);
    } else if (e_edge->get()) {
        offset_yaw = vz_ramp->Calc(current_speed_offset);
    } else if (q_edge->get()) {
        offset_yaw = vz_ramp->Calc(-current_speed_offset);
    } else {
        if (offset_yaw > 0) {
            offset_yaw = vz_ramp->Calc(-current_speed_offset);
        } else if (offset_yaw < 0) {
            offset_yaw = vz_ramp->Calc(current_speed_offset);
        }
    }
    if (abs(vx_set_org) < 0.05f) {
        vx_set_org = 0;
    }
    if (abs(vy_set_org) < 0.05f) {
        vy_set_org = 0;
    }
    if (abs(offset_yaw) < 0.05f) {
        offset_yaw = 0;
    }

#endif

}

void init_chassis() {
    // 添加can bridge，注册本机ID
    can_bridge = new communication::CanBridge(can1, 0x51);
    // 添加can bridge的底盘控制器
    chassis = new control::ChassisCanBridgeSender(can_bridge, 0x52);
    // 设置底盘各目标的寄存器id
    chassis->SetChassisRegId(0x70, 0x71, 0x72, 0x73);
    chassis->Disable();
}

void kill_chassis() {
    chassis->Disable();
}