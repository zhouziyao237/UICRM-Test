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

#include "stdint.h"

#ifdef UNIT_TEST
// make these visible to unit test
#define STATIC_UNIT_TESTED
#define STATIC_INLINE_UNIT_TESTED
#define INLINE_UNIT_TESTED
#define UNIT_TESTED
#else
#define STATIC_UNIT_TESTED static
#define STATIC_INLINE_UNIT_TESTED static inline
#define INLINE_UNIT_TESTED inline
#define UNIT_TESTED
#endif

// FAST_CODE_NOINLINE definition for STM32 HAL
#if defined(__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) || \
    defined(__GNUC__)
// ARM Compiler V4/V5 and V6 & GNU Compiler
#define FAST_CODE_NOINLINE __attribute__((section(".RamFunc"), noinline))
#elif defined(__ICCARM__)
// ICCARM Compiler
#define FAST_CODE_NOINLINE _Pragma("location=\".RamFunc\"") __attribute__((noinline))
#else
#define FAST_CODE_NOINLINE __attribute__((noinline))
#endif

typedef int32_t timeDelta_t;
// millisecond time
typedef uint32_t timeMs_t;
// microsecond time
#ifdef USE_64BIT_TIME
typedef uint64_t timeUs_t;
#define TIMEUS_MAX UINT64_MAX
#else
typedef uint32_t timeUs_t;
#define TIMEUS_MAX UINT32_MAX
#endif

static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) {
    return static_cast<timeDelta_t>(a - b);
}
static inline int32_t cmpTimeCycles(uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a - b);
}

#if __GNUC__ > 6
#define FALLTHROUGH ;__attribute__ ((fallthrough))
#else
#define FALLTHROUGH do {} while(0)
#endif

#define TASK_PERIOD_HZ(hz) (1000 / (hz))
