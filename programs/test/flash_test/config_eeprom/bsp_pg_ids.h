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

#define MotorPID_Version 1
#define PG_MotorPID_PROFILE 1 // struct OK, CF differences

#endif //BSP_PG_IDS_H
