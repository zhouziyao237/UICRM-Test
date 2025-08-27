/*
 * Betaflight / Cleanflight Derivative Work
 * ----------------------------------------
 * Original code: Copyright (c) 2011-2025 Betaflight & Cleanflight contributors
 * Derivative work: Copyright (c) 2025 BNU-HKBU UIC RoboMaster
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

//
// Created by Administrator on 2025/8/14.
//

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ARMED = (1 << 0),
    WAS_EVER_ARMED = (1 << 1),
    WAS_ARMED_WITH_PREARM = (1 << 2)
} armingFlag_e;

extern uint8_t armingFlags;

#define DISABLE_ARMING_FLAG(mask) (armingFlags &= ~(mask))
#define ENABLE_ARMING_FLAG(mask) (armingFlags |= (mask))
#define ARMING_FLAG(mask) (armingFlags & (mask))

// todo: 后面改成RM的
typedef enum {
    ARMING_DISABLED_NO_GYRO = (1 << 0),
    ARMING_DISABLED_FAILSAFE = (1 << 1),
    ARMING_DISABLED_RX_FAILSAFE = (1 << 2),
    ARMING_DISABLED_BAD_RX_RECOVERY = (1 << 3),
    ARMING_DISABLED_BOXFAILSAFE = (1 << 4),
    ARMING_DISABLED_RUNAWAY_TAKEOFF = (1 << 5),
    ARMING_DISABLED_CRASH_DETECTED = (1 << 6),
    ARMING_DISABLED_THROTTLE = (1 << 7),
    ARMING_DISABLED_ANGLE = (1 << 8),
    ARMING_DISABLED_BOOT_GRACE_TIME = (1 << 9),
    ARMING_DISABLED_NOPREARM = (1 << 10),
    ARMING_DISABLED_LOAD = (1 << 11),
    ARMING_DISABLED_CALIBRATING = (1 << 12),
    ARMING_DISABLED_CLI = (1 << 13),
    ARMING_DISABLED_CMS_MENU = (1 << 14),
    ARMING_DISABLED_BST = (1 << 15),
    ARMING_DISABLED_MSP = (1 << 16),
    ARMING_DISABLED_PARALYZE = (1 << 17),
    ARMING_DISABLED_GPS = (1 << 18),
    ARMING_DISABLED_RESC = (1 << 19),
    ARMING_DISABLED_RPMFILTER = (1 << 20),
    ARMING_DISABLED_REBOOT_REQUIRED = (1 << 21),
    ARMING_DISABLED_DSHOT_BITBANG = (1 << 22),
    ARMING_DISABLED_ACC_CALIBRATION = (1 << 23),
    ARMING_DISABLED_MOTOR_PROTOCOL = (1 << 24),
    ARMING_DISABLED_ARM_SWITCH = (1 << 25),
    // Needs to be the last element, since it's always activated if one of the others is active
    // when arming
} armingDisableFlags_e;

void setArmingDisabled(armingDisableFlags_e flag);
void unsetArmingDisabled(armingDisableFlags_e flag);
bool isArmingDisabled(void);
armingDisableFlags_e getArmingDisableFlags(void);
