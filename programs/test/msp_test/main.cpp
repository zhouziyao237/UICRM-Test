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

#include "Common/config.h"
#include "Device/bsp_Serial/DeviceSerial.h"

#include "Task/RXTask/rxtask.h"
#include "Task/MSPTask/msptask.h"
#include "Task/RFSTask/refereetask.h"
#include "bsp_PG/bsp_pg.h"
#include "bsp_print.h"
#include "bsp_serial.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
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

    RMFC::serialInit(false, RMFC::SERIAL_PORT_NONE);
    InitRX();
    InitRFS();
    InitMSP();

    osDelay(100);
}

void RM_RTOS_Threads_Init(void) {
    MspTaskHandle = osThreadNew(UpdataMsptask, nullptr, &msp_thread_attr);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    osDelay(50);
    while (true) {
        set_cursor(0, 0);
        clear_screen();

        char buf[512];
        vTaskList(buf);
        print("vTaskList: \r\n ");
        print("%s\r\n", buf);
        osDelay(200);
    }
}
