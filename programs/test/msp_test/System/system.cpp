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
// Created by Administrator on 2025/8/13.
//

#include "system.h"
#include "bsp_os.h"
#include "cmsis_os2.h"

static void indicate(uint8_t count, uint16_t duration)
{
    UNUSED(duration);
    UNUSED(count);

    // if (count) {
    //     LED1_ON;
    //     LED0_OFF;
    //
    //     while (count--) {
    //         LED1_TOGGLE;
    //         LED0_TOGGLE;
    //         BEEP_ON;
    //         delay(duration);
    //
    //         LED1_TOGGLE;
    //         LED0_TOGGLE;
    //         BEEP_OFF;
    //         delay(duration);
    //     }
    // }
}

void indicateFailure(failureMode_e mode, int codeRepeatsRemaining)
{
    while (codeRepeatsRemaining--) {
        indicate(WARNING_FLASH_COUNT, WARNING_FLASH_DURATION_MS);

        osDelay(WARNING_PAUSE_DURATION_MS);

        indicate(mode + 1, WARNING_CODE_DURATION_LONG_MS);

        osDelay(1000);
    }
}

void failureMode(failureMode_e mode)
{
    indicateFailure(mode, 10);

#ifdef DEBUG
    systemReset();
#else
    systemResetToBootloader();
#endif
}

void systemReset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

