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
#define PACK 0x0401
#define TARGET_ANGLE 0x0402
#define NO_TARGET_FLAG 0x0403
#define SHOOT_CMD 0x0404
#define ROBOT_MOVE_SPEED 0x0405

/* Status send to minipc */
#define ROBOT_POWER_HEAT_HP_UPLOAD 0x0501
#define GIMBAL_CURRENT_STATUS 0x0502
#define CHASSIS_CURRENT_STATUS 0x0503
#define AUTOAIM_ENABLE 0x0504
#define ROBOT_STATUS_UPLOAD 0x0505
