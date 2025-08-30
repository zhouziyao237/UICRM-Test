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
// Created by Administrator on 2025/8/27.
//

#include "rxtask.h"

#include "Module/RX/rx.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "bsp_thread.h"
#include "cmsis_os2.h"
#include "platform.h"
#include "target.h"

static rxState_e rxState = RX_STATE_CHECK;
osTimerId_t updateRxMainTimer;
bsp::EventThread* RXcallbackthread_ = nullptr;

bool UpdateRxMainInProgress(void) {
    return (rxState != RX_STATE_CHECK);
}

void taskUpdateRxMain(void* args) {
    UNUSED(args);
    // timeUs_t currentTimeUs = bsp::GetHighresTickMicroSec();

    switch (rxState) {
        default:
        case RX_STATE_CHECK:
            // if (!processRx(currentTimeUs)) {
            //     rxState = RX_STATE_CHECK;
            //     break;
            // }
            rxState = RX_STATE_MODES;
            break;

        case RX_STATE_MODES:
            // processRxModes(currentTimeUs);
            rxState = RX_STATE_UPDATE;
            break;

        case RX_STATE_UPDATE:
            // updateRcCommands sets rcCommand, which is needed by updateAltHoldState
            // and updateSonarAltHoldState
            // updateRcCommands();
            // updateArmingStatus();

#ifdef USE_USB_CDC_HID
            if (!ARMING_FLAG(ARMED)) {
                sendRcDataToHid();
            }
#endif
            rxState = RX_STATE_CHECK;
            break;
    }
}

static void callbackFunction(void* args) {
    UNUSED(args);
#if defined(DEBUG_RX)
    print("Timer callback fired!\r\n");
#endif

    RXcallbackthread_->Set();
}

bool InitRX() {
    RMFC::RX::rxInit();
    bsp::thread_init_t init = {
        .func = taskUpdateRxMain,
        .args = nullptr,
        .attr = rx_thread_attr,
    };
    RXcallbackthread_ = new bsp::EventThread(init);
    updateRxMainTimer = osTimerNew(callbackFunction, osTimerPeriodic, nullptr, &rx_timer_attr);
    osStatus_t status = osTimerStart(updateRxMainTimer, 2000);
    if (status != osOK) {
        delete RXcallbackthread_;
        return false;
    }
    return true;
}
