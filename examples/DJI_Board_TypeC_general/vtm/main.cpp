/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "main.h"

#include "bsp_print.h"
#include "cmsis_os.h"
#include "vtm.h"

static bsp::UART* uart = nullptr;
static remote::VTM* vtm;

void RM_RTOS_Init(void) {
    print_use_uart(&huart1, true, 921600);
    uart = new bsp::UART(&huart6);
    uart->SetBaudrate(912600);
    uart->SetupRx(100, true);
    vtm = new remote::VTM(uart);
    print("VTM initialized on UART6\r\n");
    HAL_Delay(100);
}

void RM_RTOS_Default_Task(const void* arguments) {
    UNUSED(arguments);

    while (true) {
        set_cursor(0, 0);
        clear_screen();

        // 添加连接状态检测
        bool is_connected = vtm->IsOnline();
        print("VTM Status: %s | Updates: %lu\r\n",
              is_connected ? "CONNECTED" : "DISCONNECTED", vtm->timestamp);

        print(
            "CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d \r\n"
            "CH4: %-4d CH5: %-4d GSWL: %d PBUT: %d \r\n"
            "Custom: L=%d R=%d\r\n",  // 添加自定义按键显示
            vtm->ch0, vtm->ch1, vtm->ch2, vtm->ch3,
            vtm->ch4, vtm->ch5, vtm->gear_switch, vtm->pause_button,
            vtm->custom_left, vtm->custom_right);  // 添加自定义按键值

        print("W: %d A: %d S: %d D: %d\r\n", vtm->keyboard.bit.W, vtm->keyboard.bit.A,
              vtm->keyboard.bit.S, vtm->keyboard.bit.D);
        print("Q: %d E: %d R: %d F: %d G: %d\r\n", vtm->keyboard.bit.Q, vtm->keyboard.bit.E,
              vtm->keyboard.bit.R, vtm->keyboard.bit.F, vtm->keyboard.bit.G);
        print("Z: %d X: %d C: %d V: %d B: %d\r\n", vtm->keyboard.bit.Z, vtm->keyboard.bit.X,
              vtm->keyboard.bit.C, vtm->keyboard.bit.V, vtm->keyboard.bit.B);
        print("SHIFT: %d CTRL: %d\r\n", vtm->keyboard.bit.SHIFT, vtm->keyboard.bit.CTRL);
        print("Mouse: x: %d y: %d z: %d l: %d r: %d\r\n", vtm->mouse.x, vtm->mouse.y,
              vtm->mouse.z, vtm->mouse.l, vtm->mouse.r);

        osDelay(100);  // 增加延迟以便观察
    }
}
