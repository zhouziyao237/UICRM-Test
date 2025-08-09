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
#include "bsp_pg/bsp_eeprom.h"
#include "bsp_pg/bsp_pg.h"
#include "bsp_print.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "chassis_task.h"
#include "system.h"

void RM_RTOS_Init() {
    HAL_Delay(100);
    print_use_uart(&huart8, true, 912600);

    osDelay(300);
    init_buzzer();
}

void RM_MODEL_LODE() {

    clear_screen();
    set_cursor(0, 0);

    sys_init();

    InitMotor(motors);
    InitMotor_PID(motors);
    ChassisInit(motors, nullptr, Chissis_ControlMode, chassis_remote);
    return;
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    RM_MODEL_LODE();
    osDelay(500);
    Buzzer_Sing(Mario);
    print("system OK\r\n");
    while (true) {
        clear_screen();
        set_cursor(0, 0);
        print(
            "CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d \r\nSWL: %d SWR: %d "
            "TWL: %d "
            "@ %d "
            "ms\r\n",
            dbus->ch0, dbus->ch1, dbus->ch2, dbus->ch3, dbus->swl, dbus->swr, dbus->ch4,
            dbus->timestamp);
        ChassisUpdate(Chissis_ControlMode, chassis_remote);
        osDelay(10);
    }
}
