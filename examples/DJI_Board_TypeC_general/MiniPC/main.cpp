// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
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
// Created by Administrator on 2026/1/14.
//
#include "main.h"

#include "bsp_print.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "protocol.h"

namespace {
constexpr uint32_t kTransmitPeriodMs = 2U;
constexpr uint32_t kReportPeriodMs = 1000U;
}

bsp::VirtualUSB* usb = nullptr;
communication::Host* host = nullptr;

void RM_RTOS_Init(void) {
    print_use_uart(&huart1, true, 921600);

    usb = new bsp::VirtualUSB();
    usb->SetupRx(300);
    usb->SetupTx(300);

    communication::Protocol_t protocol = {
        .uart = nullptr,
        .usb = usb,
    };
    host = new communication::Host(protocol);
}

void RM_RTOS_Threads_Init(void) {
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);
    print("sys run");
    osDelay(500);

    uint32_t tx_frames = 0U;
    uint32_t tx_bytes = 0U;
    uint32_t report_tick = HAL_GetTick();

    while (true) {
        host->gimbal_current_status.current_imu_pitch = 1;
        host->gimbal_current_status.current_imu_roll = 22;
        host->gimbal_current_status.current_imu_yaw = 33;
        host->gimbal_current_status.robot_id = 0U;
        host->gimbal_current_status.shooter_id = 0U;

        const communication::package_t package =
            host->Transmit(communication::GIMBAL_CURRENT_STATUS);
        ++tx_frames;
        tx_bytes += static_cast<uint32_t>(package.length);

        const uint32_t now = HAL_GetTick();
        if (now - report_tick >= kReportPeriodMs) {
            const uint32_t last_rx = host->GetLastUptime();
            if (last_rx == 0U) {
                print("usb link tx_frames=%lu tx_bytes=%lu rx=NEVER\r\n",
                      static_cast<unsigned long>(tx_frames),
                      static_cast<unsigned long>(tx_bytes));
            } else {
                print("usb link tx_frames=%lu tx_bytes=%lu rx=%s rx_age_ms=%lu\r\n",
                      static_cast<unsigned long>(tx_frames),
                      static_cast<unsigned long>(tx_bytes),
                      host->IsOnline() ? "ONLINE" : "OFFLINE",
                      static_cast<unsigned long>(now - last_rx));
            }
            tx_frames = 0U;
            tx_bytes = 0U;
            report_tick = now;
        }

        osDelay(kTransmitPeriodMs);
    }
}
