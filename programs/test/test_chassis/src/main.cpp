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
#include "bsp_print.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "chassis_task.h"

#include "config_eeprom/bsp_eeprom.h"
#include "config_eeprom/bsp_pg.h"
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
    system_state= SYSTEM_INIT;
        // 打印EEPROM配置信息
    print("EEPROM start: 0x%08lX\r\n", (uint32_t)__config_start);
    print("EEPROM end: 0x%08lX\r\n", (uint32_t)__config_end);
    print("EEPROM size: %lu bytes\r\n", (uint32_t)(__config_end - __config_start));

    // 加载EEPROM中的配置
    if (isEEPROMContentValid()) {
        system_state= SYSTEM_READCONFIG;
        print("EEPROM content is valid, loading configuration...\r\n");
        loadEEPROM();
    } else {
        system_state= SYSTEM_RESETCONFIG;
        print("EEPROM content is invalid, resetting to defaults...\r\n");
        pgResetAll();
        writeConfigToEEPROM();
        print("Configuration reset complete.\r\n");
    }

    // 计算注册的配置项数量和大小
    int count = 0;
    int total_size = 0;
    PG_FOREACH(reg) {
        count++;
        total_size += pgSize(reg);
        print("PG #%d: pgn=%d, size=%d bytes\r\n", count, pgN(reg), pgSize(reg));
        osDelay(50);
    }
    print("Total PG count: %d\r\n", count);
    print("Total PG data size: %d bytes\r\n", total_size);
    print("EEPROM used: %d bytes\r\n", getEEPROMConfigSize());

    InitMotor(motors);
    InitMotor_PID(motors);
    // InitCanBridgeChassis(motors, nullptr);
    ChassisInit(motors, nullptr, remote_mode, chassis_remote);
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
        ChassisUpdate(remote_mode, chassis_remote);
        osDelay(10);
    }
}
