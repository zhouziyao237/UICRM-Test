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
// Created by Administrator on 2025/7/30.
//

#ifndef BSP_PG_IDS_H
#define BSP_PG_IDS_H

#if (FLASH_SIZE > 64)
#define MAX_PROFILE_COUNT 3
#else
#define MAX_PROFILE_COUNT 2
#endif

/* - system config - */
#define SYS_VERSION "1.0.0"
#define SYSTEM_CONFIG_MAX_PROFILE_COUNT 1

/* - ProfileConfig - */
#define PROFILE_CONFIG_Version 1
#define PROFILE_CONFIG_PROFILE_COUNT 0
#define PG_PROFILE_CONFIG_PROFILE 1

/* - ControlModel - */
#define ControlModel_Version 1
#define ControlModel_PROFILE_COUNT 0
#define PG_ControlModel_PROFILE 2

/* - MotorPID - */
#define MotorPID_Version 1
#define MotorPID_PROFILE_COUNT 0
#define PG_MotorPID_PROFILE 3 // struct OK, CF differences

/* - MotorId - */
#define MotorId_Version 1
#define MotorId_PROFILE_COUNT 0
#define PG_MotorId_PROFILE 4 // struct OK, CF differences

/* - CanBridge - */
#define CanBridge_Version 1
#define CanBridge_PROFILE_COUNT 0
#define PG_CanBridge_PROFILE 5 // struct OK, CF differences

/* - RemoteDbus - */
#define RemoteDbus_Version 1
#define RemoteDbus_PROFILE_COUNT 0
#define PG_RemoteDbus_PROFILE 6 // struct OK, CF differences

/* - RemoteSbus - */
#define RemoteSbus_Version 1
#define RemoteSbus_PROFILE_COUNT 0
#define PG_RemoteSbus_PROFILE 7 // struct OK, CF differences

/* - SuperCap - */
#define SuperCap_Version 1
#define SuperCap_PROFILE_COUNT 0
#define PG_SuperCap_PROFILE 8 // struct OK, CF differences

/* - FEATURE - */

#define PG_FEATURE_CONFIG 19 // just has enabledFeatures

#endif //BSP_PG_IDS_H
