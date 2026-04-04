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

// 底盘相对于云台的局部偏角（[-PI, PI]），follow 模式用它让 chassis 跟上 gimbal
float chassis_rel_yaw = 0.0f;

// ----------------------------- Battery Voltage -----------------------------
bsp::BatteryVol* battery_vol = nullptr;

// ----------------------------- Speed Limits -----------------------------
const float chassis_max_xy_speed = 2 * PI * 30;
const float chassis_max_t_speed = 2 * PI * 20;

// ----------------------------- Ease -----------------------------
static Ease chassis_ease_vx(0, 0.9f);
static Ease chassis_ease_vy(0, 0.9f);
static Ease chassis_ease_vt(0, 0.9f);

// ----------------------------- Helper: Get normalized RC value -----------------------------
static inline float get_rc_channel(uint8_t channel) {
    return rcData[channel] / VTM_ROCKER_MAX;
}

static inline float wrap_pi(float angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

// ----------------------------- Init / Kill -----------------------------
void InitChassis() {
    battery_vol = new bsp::BatteryVol(&hadc3, ADC_CHANNEL_8, 1, ADC_SAMPLETIME_3CYCLES);

    can_bridge = new communication::CanBridge(can1, 0x51);
    chassis = new control::ChassisCanBridgeSender(can_bridge, 0x52);
    chassis->SetChassisRegId(0x70, 0x71, 0x72, 0x73);
    chassis->Disable();
}

void kill_chassis() {
    chassis->Disable();
}

// ----------------------------- Main Task -----------------------------
void ChassisTask(void* arg) {
    UNUSED(arg);

    kill_chassis();
    osDelay(1000);

    while (ChassisMode == CKILLMODE) {
        kill_chassis();
        osDelay(5);
    }

    while (!ahrs->IsCailbrated()) {
        osDelay(1);
    }

    chassis->Enable();

    while (true) {
        if (ChassisMode == CKILLMODE) {
            kill_chassis();
            while (ChassisMode == CKILLMODE) {
                osDelay(5);
            }
            chassis->Enable();
            continue;
        }

        // ----------------- 获取整车速度指令（范围 -1~1）-----------------
        float car_vx = 0.0f, car_vy = 0.0f, car_vt = 0.0f;

        if (fabs(rcData[CHASSIS_FB_CH]) > 0.01f ||
            fabs(rcData[CHASSIS_LR_CH]) > 0.01f ||
            fabs(rcData[CHASSIS_ROT_CH]) > 0.01f) {
            float fb = get_rc_channel(CHASSIS_FB_CH);
            float lr = get_rc_channel(CHASSIS_LR_CH);
            float rot = get_rc_channel(CHASSIS_ROT_CH);

            if (CHASSIS_FB_REV) fb = -fb;
            if (CHASSIS_LR_REV) lr = -lr;
            if (CHASSIS_ROT_REV) rot = -rot;

            car_vx = fb;
            car_vy = lr;
            car_vt = rot;
        } else {
            const float keyboard_speed = (rcData[CSPEEDUP] > 0.5f) ? 1.0f : 0.5f;
            const float keyboard_spin_speed = 1.0f;
            car_vx = (rcData[CRIGHT] - rcData[CLEFT]) * keyboard_speed;
            car_vy = (rcData[CFORWARD] - rcData[CBACKWARD]) * keyboard_speed;
            car_vt = (rcData[CROTATERIGHT] - rcData[CROTATELEFT]) * keyboard_spin_speed;
        }

        // ----------------- 云台相对底盘的局部 yaw -----------------
        float gimbal_local_yaw = gimbal->getYawByMotor();
        chassis_rel_yaw = wrap_pi(gimbal_local_yaw);

        // ----------------- 以“云台朝向/用户视角”为基准的平移映射 -----------------
        // 在 follow / spin 下使用这组速度，可以保证用户推杆手感一致。
        float sin_yaw = arm_sin_f32(gimbal_local_yaw);
        float cos_yaw = arm_cos_f32(gimbal_local_yaw);
        float follow_vx = cos_yaw * car_vx + sin_yaw * car_vy;
        float follow_vy = -sin_yaw * car_vx + cos_yaw * car_vy;

        // 默认给 0，后面按模式覆盖
        chassis_vx = 0.0f;
        chassis_vy = 0.0f;
        chassis_vt = 0.0f;

        // ----------------- 模式处理 -----------------
        switch (ChassisMode) {
            case CMANUALMODE:
                // 手动：完全按底盘本体坐标控制
                chassis_vx = car_vx;
                chassis_vy = car_vy;
                chassis_vt = car_vt;
                break;

            case CFOLLOWMODE: {
                // follow：用户还是在控云台；底盘去追云台局部角，让 chassis 跟 gimbal 转
                chassis_vx = follow_vx;
                chassis_vy = follow_vy;

                static control::ConstrainedPID* chassis_vt_pid =
                    new control::ConstrainedPID(1.0f / PI, 0.0f, 0.0f, 0.5f, 1.0f);
                const float angle_threshold = 0.02f;

                float error = -chassis_rel_yaw;  // 目标是让相对角回到 0
                if (fabs(error) < angle_threshold) error = 0.0f;
                chassis_vt = chassis_vt_pid->ComputeOutput(error);
                break;
            }

            case CSPINMODE:
                // spin：底盘旋转，但云台维持用户操控手感不变。
                // 平移仍按云台/用户视角映射；旋转沿用用户给的转速。
                // 若你想“进入 spin 模式自动恒定自转”，把 car_vt 改成固定常量即可。
                chassis_vx = follow_vx;
                chassis_vy = follow_vy;
                chassis_vt = car_vt;
                break;

            case CAUTOAIMODE:
            default:
                chassis_vx = follow_vx;
                chassis_vy = follow_vy;
                chassis_vt = 0.0f;
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

        osDelay(5);
    }
}
