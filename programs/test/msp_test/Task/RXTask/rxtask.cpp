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
#include "bsp_tim.h"
#include "cmsis_os2.h"
#include "platform.h"
#include "target.h"

static rxState_e rxState = RX_STATE_CHECK;
bsp::Timer* rxTimer = nullptr;
bsp::EventThread* RXcallbackthread_ = nullptr;
bsp::InitTIMConfig RxTimConfig = {
    .IRQn = TIM1_UP_TIM10_IRQn,
    .PreemptPriority = 720 - 1,
    .SubPriority = 100 - 1,
    .CounterMode = TIM_COUNTERMODE_UP,
    .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE,
};

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
    // todo: I don't know why the program keeps stuck here
    // print("Timer callback fired!\r\n");
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
    rxTimer = new bsp::Timer(&htim10, RxTimConfig, TASK_PERIOD_HZ(33));
    HAL_StatusTypeDef status = rxTimer->TimerInit();
    rxTimer->RegisterCallback(callbackFunction, nullptr);
    rxTimer->Start_IT();
    if (status != HAL_OK) {
        delete RXcallbackthread_;
        return false;
    }
    return true;
}
