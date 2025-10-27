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
// Created by Administrator on 2025/8/1.
//

#include "main.h"

#include "../../../boards/platform/stm32h7/include/bsp_os.h"
#include "Common/config.h"
#include "Device/bsp_Serial/DeviceSerial.h"
#include "FreeRTOS.h"
#include "Module/dbus/dbus_channels.h"
#include "Task/MSPTask/msptask.h"
#include "Task/MiniPCTask/minipctask.h"
#include "Task/RFSTask/refereetask.h"
#include "Task/RXTask/rxtask.h"
#include "bsp_PG/bsp_pg.h"
#include "bsp_print.h"
#include "bsp_serial.h"
#include "cmsis_os2.h"
#include "task.h"

void RM_RTOS_Init() {
    pgResetAll();

#ifdef USE_DEBUGUART
    print_use_uart(USE_DEBUGUART, false, 912600);
#endif

    print("EEPROM start: 0x%08lX\r\n", (uint32_t)__config_start);
    print("EEPROM end: 0x%08lX\r\n", (uint32_t)__config_end);
    print("EEPROM size: %lu bytes\r\n", (uint32_t)(__config_end - __config_start));

    initEEPROM();
    ensureEEPROMStructureIsValid();
    readEEPROM();

    bsp::SetHighresClockTimer(&htim7);
    RMFC::serialInit(false, RMFC::SERIAL_PORT_NONE);
    InitRX();
    InitMSP();
    InitRFS();
    InitMiniPC();

    osDelay(500);
}

void RM_RTOS_Threads_Init(void) {
    MspTaskHandle = osThreadNew(UpdataMsptask, nullptr, &MspUpData_thread_attr);
    RFSTaskHandle = osThreadNew(UpdataRefereetask, nullptr, &RefereeUpData_thread_attr);
    MiniPCTaskHandle = osThreadNew(UpdataMiniPCtask, nullptr, &MiniPCUpData_thread_attr);
    RxTaskHandle = osThreadNew(UpdataRxTask, nullptr, &RxUpData_thread_attr);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    osDelay(50);
    while (true) {
        set_cursor(0, 0);
        clear_screen();

        char buf[512];
        vTaskList(buf);
        print("vTaskList: \r\n");
        print("%s\r\n", buf);

        // print("W: %d A: %d S: %d D: %d\r\n", RMFC::RX::Client_ChannelsData->keyboard.bit.W,
        //       RMFC::RX::Client_ChannelsData->keyboard.bit.A,
        //       RMFC::RX::Client_ChannelsData->keyboard.bit.S,
        //       RMFC::RX::Client_ChannelsData->keyboard.bit.D);
        // print("Q: %d E: %d R: %d F: %d G: %d\r\n", RMFC::RX::Client_ChannelsData->keyboard.bit.Q,
        //       RMFC::RX::Client_ChannelsData->keyboard.bit.E,
        //       RMFC::RX::Client_ChannelsData->keyboard.bit.R,
        //       RMFC::RX::Client_ChannelsData->keyboard.bit.F,
        //       RMFC::RX::Client_ChannelsData->keyboard.bit.G);
        // print("Z: %d X: %d C: %d V: %d B: %d\r\n", RMFC::RX::Client_ChannelsData->keyboard.bit.Z,
        //       RMFC::RX::Client_ChannelsData->keyboard.bit.X,
        //       RMFC::RX::Client_ChannelsData->keyboard.bit.C,
        //       RMFC::RX::Client_ChannelsData->keyboard.bit.V,
        //       RMFC::RX::Client_ChannelsData->keyboard.bit.B);
        // print("SHIFT: %d CTRL: %d\r\n", RMFC::RX::Client_ChannelsData->keyboard.bit.SHIFT,
        //       RMFC::RX::Client_ChannelsData->keyboard.bit.CTRL);
        // print("Mouse: x: %d y: %d z: %d l: %d r: %d\r\n", RMFC::RX::Client_ChannelsData->mouse.x,
        //       RMFC::RX::Client_ChannelsData->mouse.y, RMFC::RX::Client_ChannelsData->mouse.z,
        //       RMFC::RX::Client_ChannelsData->mouse.l, RMFC::RX::Client_ChannelsData->mouse.r);

        osDelay(200);
    }
}
