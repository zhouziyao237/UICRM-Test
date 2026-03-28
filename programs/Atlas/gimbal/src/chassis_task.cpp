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

#include "chassis_task.h"
#include "gimbal_task.h"            // yaw_motor, gimbal_param
#include "imu_task.h"               // ahrs, INS_Angle

// ----------------------------- Task handle -----------------------------
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTaskAttribute = {
    .name = "chassisTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0
};

// ----------------------------- CanBridge -----------------------------
communication::CanBridge* can_bridge = nullptr;
control::ChassisCanBridgeSender* chassis = nullptr;

// ----------------------------- Motion Data -----------------------------
float chassis_vx = 0;
float chassis_vy = 0;
float chassis_vt = 0;

// ----------------------------- Battery Voltage -----------------------------
bsp::BatteryVol* battery_vol = nullptr;

// ----------------------------- Speed Limits -----------------------------
const float chassis_max_xy_speed = 2 * PI * 30;   // 线速度上限 10圈/s
const float chassis_max_t_speed = 2 * PI * 20;     // 旋转速度上限 5圈/s

// ----------------------------- Ease -----------------------------
static Ease chassis_ease_vx(0, 0.9f);
static Ease chassis_ease_vy(0, 0.9f);
static Ease chassis_ease_vt(0, 0.9f);

// ----------------------------- Helper: Get normalized RC value -----------------------------
static inline float get_rc_channel(uint8_t channel) {
    return rcData[channel] / VTM_ROCKER_MAX;
}

// ----------------------------- Init / Kill -----------------------------
void InitChassis() {
    // 初始化电池电压测量（ADC3 通道8）
    battery_vol = new bsp::BatteryVol(&hadc3, ADC_CHANNEL_8, 1, ADC_SAMPLETIME_3CYCLES);

    // 创建 CanBridge（发送ID 0x51）
    can_bridge = new communication::CanBridge(can1, 0x51);
    // 创建底盘控制器（接收ID 0x52）
    chassis = new control::ChassisCanBridgeSender(can_bridge, 0x52);
    // 设置寄存器ID
    chassis->SetChassisRegId(0x70, 0x71, 0x72, 0x73);
    chassis->Disable();
}

void kill_chassis() {
    chassis->Disable();
}

// ----------------------------- Main Task -----------------------------
void ChassisTask(void* arg) {
    UNUSED(arg);

    // 初始禁用底盘，等待条件
    kill_chassis();
    osDelay(1000);

    // 等待遥控器退出急停模式（ChassisMode 由 SysTask 根据遥控器设定）
    while (ChassisMode == CKILLMODE) {
        kill_chassis();
        osDelay(5);
    }

    // 等待 IMU 校准完成
    while (!ahrs->IsCailbrated()) {
        osDelay(1);
    }

    // 使能底盘
    chassis->Enable();

    while (true) {
        // ----------------- 急停检查（再次确保） -----------------
        if (ChassisMode == CKILLMODE) {
            kill_chassis();
            while (ChassisMode == CKILLMODE) {
                osDelay(5);
            }
            chassis->Enable();
            continue;
        }

        // ----------------- 获取整车速度指令（范围 -1~1）-----------------
        float car_vx = 0, car_vy = 0, car_vt = 0;

        // 优先使用摇杆（有非零输入）
        if (fabs(rcData[CHASSIS_FB_CH]) > 0.01f ||
            fabs(rcData[CHASSIS_LR_CH]) > 0.01f ||
            fabs(rcData[CHASSIS_ROT_CH]) > 0.01f) {
            float fb = get_rc_channel(CHASSIS_FB_CH);
            float lr = get_rc_channel(CHASSIS_LR_CH);
            float rot = get_rc_channel(CHASSIS_ROT_CH);

            // 反向标志（来自 sys.h）
            if (CHASSIS_FB_REV) fb = -fb;
            if (CHASSIS_LR_REV) lr = -lr;
            if (CHASSIS_ROT_REV) rot = -rot;

            car_vx = fb;
            car_vy = lr;
            car_vt = rot;
        } else {
            // 键盘模式
            const float keyboard_speed = (rcData[CSPEEDUP] > 0.5f) ? 1.0f : 0.5f;
            const float keyboard_spin_speed = 1.0f;
            car_vx = (rcData[CRIGHT] - rcData[CLEFT]) * keyboard_speed;
            car_vy = (rcData[CFORWARD] - rcData[CBACKWARD]) * keyboard_speed;
            car_vt = (rcData[CROTATERIGHT] - rcData[CROTATELEFT]) * keyboard_spin_speed;
        }

        // ----------------- 坐标变换（云台相对底盘角度）-----------------
        float chassis_target_diff = yaw_motor->GetTheta() - gimbal_param->yaw_offset_;

        float sin_yaw = arm_sin_f32(chassis_target_diff);
        float cos_yaw = arm_cos_f32(chassis_target_diff);
        chassis_vx = cos_yaw * car_vx + sin_yaw * car_vy;
        chassis_vy = -sin_yaw * car_vx + cos_yaw * car_vy;
        chassis_vt = 0;

        // ----------------- 模式特殊处理 -----------------
        switch (ChassisMode) {
            case CMANUALMODE:
                // 手动模式：直接使用整车指令（以云台为基准）
                chassis_vx = car_vx;
                chassis_vy = car_vy;
                chassis_vt = car_vt;
                break;
            case CFOLLOWMODE: {
                // 跟随模式：底盘旋转跟随云台方向
                static control::ConstrainedPID* chassis_vt_pid =
                    new control::ConstrainedPID(2 / (2 * PI), 0, 0, 0.5f, 1.0f);
                const float angle_threshold = 0.02f;
                float error = chassis_target_diff;
                if (fabs(error) < angle_threshold) error = 0;
                chassis_vt = chassis_vt_pid->ComputeOutput(error);
                break;
            }
            case CSPINMODE: {
                +
                // 小陀螺模式：旋转速度累加
                static float spin_speed = 0;
                spin_speed += car_vt * 0.1f;
                spin_speed = clip<float>(spin_speed, -6.0f, 6.0f);
                chassis_vt = spin_speed;
                break;
            }
            case CAUTOAIMODE:
            default:
                // 其他模式（自瞄等）保持变换后的速度
                break;
        }

        // ----------------- 速度限幅 + 平滑 -----------------
        chassis_vx = clip<float>(chassis_vx, -1.0f, 1.0f) * chassis_max_xy_speed;
        chassis_vy = clip<float>(chassis_vy, -1.0f, 1.0f) * chassis_max_xy_speed;
        chassis_vt = clip<float>(chassis_vt, -1.0f, 1.0f) * chassis_max_t_speed;

        chassis_vx = chassis_ease_vx.Calc(chassis_vx);
        chassis_vy = chassis_ease_vy.Calc(chassis_vy);
        chassis_vt = chassis_ease_vt.Calc(chassis_vt);

        // ----------------- 发送速度 -----------------
        chassis->SetSpeed(chassis_vx, chassis_vy, chassis_vt);

        // ----------------- 功率控制（使用板载电压） -----------------
#ifdef HAS_REFEREE
        uint8_t buffer_percent = Referee_data->power_heat_data.chassis_power_buffer * 100 / 60;
        uint8_t max_watt = Referee_data->game_robot_status.chassis_power_limit;
#else
        uint8_t buffer_percent = 50;
        uint8_t max_watt = 100;
#endif
        float voltage = battery_vol ? battery_vol->GetBatteryVol() : 0.0f;
        chassis->UpdatePower(true, max_watt, voltage, buffer_percent);

        // ----------------- 控制周期 -----------------
        osDelay(5);
    }
}