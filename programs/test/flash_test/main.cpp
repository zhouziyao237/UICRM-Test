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
// Created by Administrator on 2025/7/25.
//

// main.cpp

#include "main.h"
#include "bsp_print.h"
#include "config_eeprom/bsp_eeprom.h"
#include "config_eeprom/bsp_pg.h"
#include "config/config.h"

void RM_RTOS_Init() {
    print_use_uart(&huart8, true, 912600);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    clear_screen();
    set_cursor(0, 0);
    print("system OK\r\n");

    // 打印EEPROM配置信息
    print("EEPROM start: 0x%08lX\r\n", (uint32_t)__config_start);
    print("EEPROM end: 0x%08lX\r\n", (uint32_t)__config_end);
    print("EEPROM size: %lu bytes\r\n", (uint32_t)(__config_end - __config_start));

    // 加载EEPROM中的配置
    if (isEEPROMContentValid()) {
        print("EEPROM content is valid, loading configuration...\r\n");
        loadEEPROM();
    } else {
        print("EEPROM content is invalid, resetting to defaults...\r\n");
        pgResetAll();
    }

    // 显示当前MotorPID配置
    print("Current MotorPID configurations:\r\n");
    for (int i = 0; i < MAX_PROFILE_COUNT; i++) {
        const MotorPID_t* pid = getMotorPID(i);
        if (pid) {
            print("Profile %d: kp=%.2f, ki=%.2f, kd=%.2f\r\n", i, pid->kp, pid->ki, pid->kd);
        }
    }

    // 修改第一个配置文件的PID参数示例
    print("Modifying MotorPID profile 0...\r\n");
    setMotorPID(0, 3000.0f, 5.0f, 10.0f);

    const MotorPID_t* modified_pid = getMotorPID(0);
    print("Modified Profile 0: kp=%.2f, ki=%.2f, kd=%.2f\r\n",
          modified_pid->kp, modified_pid->ki, modified_pid->kd);

    // 保存配置到EEPROM
    print("Saving configuration to EEPROM...\r\n");
    writeConfigToEEPROM();

    if (isEEPROMContentValid()) {
        print("Configuration saved successfully!\r\n");
    } else {
        print("Failed to save configuration!\r\n");
    }

    // 计算注册的配置项数量和大小
    int count = 0;
    int total_size = 0;
    PG_FOREACH(reg) {
        count++;
        total_size += pgSize(reg);
        print("PG #%d: pgn=%d, size=%d bytes\r\n", count, pgN(reg), pgSize(reg));
    }
    print("Total PG count: %d\r\n", count);
    print("Total PG data size: %d bytes\r\n", total_size);
    print("EEPROM used: %d bytes\r\n", getEEPROMConfigSize());
}
