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
#include "can_bridge.h"
#include "cmsis_os.h"
#include "protocol.h"

// ============================================// 第一节：硬件对象声明
// ============================================
bsp::CAN* can = nullptr;
communication::CanBridge* can_bridge = nullptr;
communication::RefereeCanProtocol* referee_protocol = nullptr;

// ============================================// 第二节：常量定义
// ============================================
constexpr uint8_t HOST_CAN_ID = 0x02;         // 本机CAN ID (Host)
constexpr uint8_t SLAVE_CAN_ID = 0x01;        // 发送方CAN ID (Slave)
constexpr uint8_t CAN_REG_ID = 0x42;          // CAN寄存器ID



// ============================================// 第四节：初始化
// ============================================
void RM_RTOS_Init() {
    osDelay(200);
    print_use_uart(&huart1, true, 915200);

    // 初始化CAN
    can = new bsp::CAN(&hcan1, true, 8);
    can_bridge = new communication::CanBridge(can, HOST_CAN_ID);

    // 初始化RefereeCanProtocol
    // 它会自动接收CAN数据并解析为Referee结构体
    referee_protocol = new communication::RefereeCanProtocol(
        can_bridge,        // CAN桥接器
        HOST_CAN_ID,       // 本机CAN节点ID
        CAN_REG_ID         // CAN寄存器ID
    );

    // 注意：RefereeCanProtocol现在使用callback+thread+switch模式
    // 数据会自动解析到对应的成员变量中，无需注册回调函数

    osDelay(300);
}

// ============================================// 第五节：主循环 - 显示接收数据
// ============================================
void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    osDelay(500);

    while (true) {
        clear_screen();
        set_cursor(0, 0);

        print("========================================\r\n");
        print("  RefereeCanProtocol Demo (Receiver)    \r\n");
        print("========================================\r\n\r\n");

        print("[Mode]   CAN Receiver (Auto Parse)\r\n");
        print("[CAN]    hcan1 <- Slave ID: 0x%02X\r\n", SLAVE_CAN_ID);
        print("[Reg]    0x%02X\r\n\n", CAN_REG_ID);

        print("---------- Robot Status ----------\r\n");
        print("Robot ID:    %d\r\n", referee_protocol->game_robot_status.robot_id);
        print("Robot Level: %d\r\n", referee_protocol->game_robot_status.robot_level);
        print("Remain HP:   %d / %d\r\n",
              referee_protocol->game_robot_status.remain_HP,
              referee_protocol->game_robot_status.max_HP);
        print("Power Limit: %d W\r\n\n",
              referee_protocol->game_robot_status.chassis_power_limit);

        print("---------- Power & Heat ----------\r\n");
        print("Voltage: %d mV\r\n", referee_protocol->power_heat_data.chassis_volt);
        print("Current: %d mA\r\n", referee_protocol->power_heat_data.chassis_current);
        print("Power:   %.2f W\r\n", referee_protocol->power_heat_data.chassis_power);
        print("Buffer:  %d J\r\n\n", referee_protocol->power_heat_data.chassis_power_buffer);

        print("---------- Game Status ----------\r\n");
        print("Game Type:     %d\r\n", referee_protocol->game_status.game_type);
        print("Game Progress: %d\r\n", referee_protocol->game_status.game_progress);
        print("Stage Time:    %d s\r\n\n", referee_protocol->game_status.stage_remain_time);

        print("---------- Shoot Data ----------\r\n");
        print("Bullet Type: %d\r\n", referee_protocol->shoot_data.bullet_type);
        print("Shooter ID:  %d\r\n", referee_protocol->shoot_data.shooter_id);
        print("Bullet Freq: %d\r\n", referee_protocol->shoot_data.bullet_freq);
        print("Bullet Speed:%.2f m/s\r\n\n", referee_protocol->shoot_data.bullet_speed);

        print("[Note] Data auto-received and parsed from CAN\r\n");

        osDelay(100);
    }
}
