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
// Created by Administrator on 2025/8/10.
//

#pragma once

#define BOARD_NAME "RMFC"

#define USE_BLACKBOX
#define USE_ACC
#define USE_MAG
#define USE_BARO
#define USE_DYN_LPF
#define USE_D_MIN
#define USE_DYN_NOTCH_FILTER
#define USE_ITERM_RELAX

#if defined(DJI_Board_TypeA_general)
#define RMFC_TARGET "DJI_Board_TypeA_general"
#elif defined(DJI_Board_TypeC_general)
#define RMFC_TARGET "DJI_Board_TypeC_general"
#elif defined(DM_MC01_general)
#define RMFC_TARGET "DM_MC01_general"
#elif defined(DM_MC02_general)
#define RMFC_TARGET "DM_MC02_general"
#else
#error "Unsupported platform"
#endif

#define U_ID_0 (*(uint32_t*)0x1fff7a10)
#define U_ID_1 (*(uint32_t*)0x1fff7a14)
#define U_ID_2 (*(uint32_t*)0x1fff7a18)

#ifndef ESPFC_REVISION
#define ESPFC_REVISION 0000000
#endif

#ifndef ESPFC_VERSION
#define ESPFC_VERSION v0.0.0
#endif

#define MAX_SUPPORTED_MOTORS 4
#define MAX_SUPPORTED_SERVOS 8
#define PID_PROCESS_DENOM_DEFAULT 1

#define FC_FIRMWARE_NAME "Betaflight"
#define FC_VERSION_MAJOR 4  // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR \
    4  // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL 0  // increment when a bug is fixed

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define FC_VERSION_STRING \
    STR(FC_VERSION_MAJOR) "." STR(FC_VERSION_MINOR) "." STR(FC_VERSION_PATCH_LEVEL)

#define MW_VERSION 1

#ifdef __cplusplus
extern "C" {
#endif

const char* const targetName = RMFC_TARGET;
const char* const targetVersion = STR(ESPFC_VERSION);
const char* const shortGitRevision = STR(ESPFC_REVISION);
const char* const buildTime = __TIME__;
const char* const buildDate = __DATE__;
const char* flightControllerIdentifier = "BTFL";
const char* boardIdentifier = "RMFC";

extern const char* const targetName;

#define GIT_SHORT_REVISION_LENGTH 7  // lower case hexadecimal digits.
extern const char* const shortGitRevision;

#define BUILD_DATE_LENGTH 11
extern const char* const buildDate;  // "MMM DD YYYY" MMM = Jan/Feb/...

#define BUILD_TIME_LENGTH 8
extern const char* const buildTime;  // "HH:MM:SS"
}
