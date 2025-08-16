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
#include "config/config.h"
#include "config/config_function.h"
#include "config_eeprom/bsp_eeprom.h"
#include "config_eeprom/bsp_pg.h"

void RM_RTOS_Init() {
    print_use_uart(&huart8, true, 912600);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    set_cursor(0, 0);
    clear_screen();
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
            print(
                "Modified Profile %d:\r\nkp=%.2f, ki=%.2f, "
                "kd=%.2f,\r\nmax_out=%.2f,\r\nmax_iout=%.2f,\r\n"
                "output_filtering_coefficient=%.2f,\r\nderivative_filtering_coefficient=%.2f,\r\n"
                "deadband=%.2f,\r\nA=%.2f,\r\nB=%.2f,\r\nmode=0x%x\r\n",
                i, pid->kp, pid->ki, pid->kd, pid->max_out, pid->max_iout,
                pid->output_filtering_coefficient, pid->derivative_filtering_coefficient,
                pid->deadband, pid->A, pid->B, pid->mode);
        }
    }

    // 修改第一个配置文件的PID参数示例
    print("Modifying MotorPID profile 0...\r\n");
    MotorPID_t MotorPids;

    for (int i = 0; i < 10; i++) {
        MotorPids.ki += 0.1f; // 递增
        setMotorPID(0, &MotorPids);
        writeConfigToEEPROM();
    }

    // // 保存配置到EEPROM
    // print("Saving configuration to EEPROM...\r\n");
    // writeConfigToEEPROM();

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
