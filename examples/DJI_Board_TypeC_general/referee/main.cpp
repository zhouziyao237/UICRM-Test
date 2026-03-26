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
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "protocol.h"

#define RX_SIGNAL (1 << 0)

communication::Referee* referee = nullptr;
bsp::UART* referee_uart = nullptr;

void RM_RTOS_Init(void) {
    print_use_uart(&huart6, true, 915200);

    referee_uart = new bsp::UART(&huart1);
    referee_uart->SetupRx(300, true);
    referee_uart->SetupTx(300, true);

    referee = new communication::Referee(referee_uart);
}

void RM_RTOS_Threads_Init(void) {
}

void RM_RTOS_Default_Task(const void* argument) {
    UNUSED(argument);

    while (true) {
        set_cursor(0, 0);
        clear_screen();
        print("---------- Received from Referee ----------\r\n");
        print("Robot ID:    %d\r\n", referee->game_robot_status.robot_id);
        print("Robot Level: %d\r\n", referee->game_robot_status.robot_level);
        print("Remain HP:   %d / %d\r\n",
              referee->game_robot_status.remain_HP,
              referee->game_robot_status.max_HP);
        print("Power Limit: %d W\r\n\n",
              referee->game_robot_status.chassis_power_limit);

        print("---------- Power & Heat ----------\r\n");
        print("Voltage: %d mV\r\n", referee->power_heat_data.chassis_volt);
        print("Current: %d mA\r\n", referee->power_heat_data.chassis_current);
        print("Power:   %.2f W\r\n", referee->power_heat_data.chassis_power);
        print("Buffer:  %d J\r\n\n", referee->power_heat_data.chassis_power_buffer);

        print("[Note] Data auto-sent via CAN when received from UART\r\n");
        osDelay(100);
    }
}