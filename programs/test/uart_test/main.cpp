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
// Created by Administrator on 2025/10/2.
//

#include "main.h"

#include "../../../boards/base/DM_MC02_general/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h"
#include "bsp_print.h"
#include "bsp_uart.h"

bsp::UART* cs_uart = nullptr;
uint8_t *data;

void Callback(void* arg) {
    UNUSED(arg);
    cs_uart->Read<true>(&data);
    // cs_uart->Write<true>(data, sizeof(&data));
}

void RM_RTOS_Init() {
    // print_use_uart(&huart8, false, 912600);
    cs_uart = new bsp::UART(&huart1);
    cs_uart->SetupRx(1, true);
    cs_uart->SetupTx(1, true);
    cs_uart->SetBaudrate(100000);
    cs_uart->RegisterCallback(Callback, nullptr);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    while (true) {
        // print("Raed data: %c \r\n", &data);
        osDelay(10);
    }
}