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
#include "MotorCanBase.h"
#include "bsp_can.h"
#include "bsp_print.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"

bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;

void RM_RTOS_Init() {
    HAL_Delay(100);
    print_use_uart(&huart6);
    can2 = new bsp::CAN(&hcan2, false);
    can1 = new bsp::CAN(&hcan1, true);


    HAL_Delay(300);
    init_buzzer();
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    osDelay(500);
    Buzzer_Sing(Mario);

    while (true) {

        osDelay(10);
    }
}
