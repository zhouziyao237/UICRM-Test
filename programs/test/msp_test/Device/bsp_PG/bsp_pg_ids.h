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


/* - FailsafeConfig - */
#define PG_FAILSAFE_CONFIG 1  // struct OK
/* - SerialConfig - */
#define PG_SERIAL_CONFIG 9
/* - FEATURE - */
#define PG_FEATURE_CONFIG 19 // just has enabledFeatures

#define PG_RX_CONFIG 24  // betaflight rxConfig_t contains different values
#define PG_RPYTA_CONFIG 25

#define PG_MODE_ACTIVATION_PROFILE 41  // array needs to be made into struct
#define PG_RX_FAILSAFE_CHANNEL_CONFIG 43
#define PG_RX_CHANNEL_RANGE_CONFIG 44

#define PG_MSP_CONFIG 557
#define PG_RFS_CONFIG 558

#endif //BSP_PG_IDS_H
