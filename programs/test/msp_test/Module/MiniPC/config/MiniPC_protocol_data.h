// Copyright (c) 2025. BNU-HKBU UIC RoboMaster
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
// Created by Administrator on 2025/10/27.
//

#pragma once

#include <stdint.h>

/* Command for Host */

/*
 * 0x0401 PACK
 * 0x0402 TARGET_ANGLE
 * 0x0403 NO_TARGET_FLAG
 * 0x0404 SHOOT_CMD
 * 0x0405 ROBOT_MOVE_SPEED
 * 0x0501 ROBOT_POWER_HEAT_HP_UPLOAD
 * 0x0502 GIMBAL_CURRENT_STATUS
 * 0x0503 CHASSIS_CURRENT_STATUS
 * 0x0504 AUTOAIM_ENABLE
 * 0x0505 ROBOT_STATUS_UPLOAD
 */

/* Control command receive from minipc */

/* ===== PACK 0x0401 ===== */
typedef struct {
    char chars[256];  // a string with maximum 256 chars
} __packed pack_t;

/* ===== TARGET_ANGLE 0x0402 ===== */
typedef struct {
    uint8_t shooter_id;
    uint8_t target_robot_id;
    float target_pitch;
    float target_yaw;
    uint8_t accuracy;  // 置信度 0-100
    uint8_t shoot_cmd;
} __packed target_angle_t;

/* ===== NO_TARGET_FLAG 0x0403 ===== */
typedef struct {
    char dummy;  // no actual meaning
} __packed no_target_flag_t;

/* ===== SHOOT_CMD 0x0404 ===== */
typedef struct {
    uint8_t shooter_id;
    uint8_t shoot_flywheel;     // 0x00 for stop, 0x01 for start
    uint8_t shoot_cmd;          // 0x00 for stop, 0x01 for shoot
    float expect_bullet_speed;  // 摩擦轮期望速度
} __packed shoot_cmd_t;

/* ===== ROBOT_MOVE_SPEED 0x0405 ===== */
typedef struct {
    float target_x;
    float target_y;
    float target_turn;
} __packed robot_move_t;

/* ===== ROBOT_POWER_HEAT_HP_UPLOAD 0x0501 20Hz ===== */
typedef struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_cooling_rate;
    uint16_t shooter_heat_limit;
    uint16_t chassis_power_limit;
    float chassis_current_power;
    float chassis_power_buffer;
} __packed robot_power_heat_hp_upload_t;

/* ===== GIMBAL_CURRENT_STATUS 0x0502 500Hz ===== */
typedef struct {
    float current_imu_pitch;
    float current_imu_roll;
    float current_imu_yaw;
    uint8_t robot_id;
    uint8_t shooter_id;
} __packed gimbal_current_status_t;

/* ===== CHASSIS_CURRENT_STATUS 0x0503 100Hz ===== */
typedef struct {
    float speed_x;
    float speed_y;
    float speed_turn;
    uint8_t chassis_enabled;
} __packed chassis_current_status_t;

/* ===== AUTOAIM_ENABLE 0x0504 ===== */
typedef struct {
    uint8_t autoaim_enabled;
} __packed autoaim_enable_t;

/* ===== ROBOT_STATUS_UPLOAD 1Hz 0x0505 ===== */
typedef struct {
    uint8_t robot_id;
    uint8_t vision_reset;      // 是否重置视觉识别
    uint8_t location_data[2];  // 裁判系统返回的位置数据，RMUL状态下为0
    uint8_t is_killed;         // 是否被击杀，裁判系统血量为0或者触发手动kill则视为被击杀
    uint8_t robot_mode;        // 机器人模式
    /*
     * 1:一般跟随
     * 2:小陀螺
     * 3:高级调试
     * 4:全自动小陀螺原地防守
     * 5:全自动哨兵
     * */
    uint8_t robot_fric_mode;
    /*
     * 1:正常
     * 0:停止
     * */
    uint8_t robot_shoot_mode;
    /*
     * 1:单发
     * 2:连发
     * 3:停止
     * */
    uint8_t supercapacitor_enabled;
    /*
     * 1:开启
     * 0:关闭
     * */
    uint8_t robot_module_online;
    /*
     * 八位Bool
     * 0:底盘
     * 1:云台
     * 2:发射机构
     * 3:IMU
     * 4-7:保留
     */
    uint8_t is_double_gimbal;
    /*
     * 1:双云台
     * 0:单云台
     * */
} __packed robot_status_upload_t;

class MiniPCdata {
  public:
    pack_t pack{};
    target_angle_t target_angle{};
    no_target_flag_t no_target_flag{};
    shoot_cmd_t shoot_cmd{};
    robot_move_t robot_move{};
    robot_power_heat_hp_upload_t robot_power_heat_hp_upload{};
    gimbal_current_status_t gimbal_current_status{};
    chassis_current_status_t chassis_current_status{};
    autoaim_enable_t autoaim_enable{};
    robot_status_upload_t robot_status_upload{};
};