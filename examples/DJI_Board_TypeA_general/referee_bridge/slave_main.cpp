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
bsp::UART* uart = nullptr;

// ============================================// 第二节：常量定义
// ============================================
constexpr uint8_t SLAVE_CAN_ID = 0x01;        // 本机CAN ID (Slave)
constexpr uint8_t HOST_CAN_ID = 0x02;         // 目标CAN ID (Host)
constexpr uint8_t CAN_REG_ID = 0x42;          // CAN寄存器ID

// 回调函数 - 处理UART接收的数据
void UARTCallback(void* args) {
    // 这里可以添加UART接收处理逻辑
    // 当从裁判系统接收到数据时，通过CAN发送
}

// ============================================// 第三节：初始化
// ============================================
void RM_RTOS_Init() {
    osDelay(200);
    print_use_uart(&huart6, true, 915200);
    uart = new bsp::UART(&huart1);
    uart->SetBaudrate(115200);
    uart->SetupRx(300);
    uart->SetupTx(300);
    uart->RegisterCallback(UARTCallback, nullptr);

    // 初始化CAN
    can = new bsp::CAN(&hcan2, true, 8);
    can_bridge = new communication::CanBridge(can, SLAVE_CAN_ID);

    // 初始化RefereeCanProtocol
    // 它会自动接收UART数据并通过CAN发送
    referee_protocol = new communication::RefereeCanProtocol(
        can_bridge,        // CAN桥接器
        SLAVE_CAN_ID,      // 本机CAN节点ID
        CAN_REG_ID         // CAN寄存器ID
    );

    // 准备发送的数据（模拟数据）
    referee_protocol->game_status.game_type = 1;
    referee_protocol->game_status.game_progress = 2;
    referee_protocol->game_status.stage_remain_time = 180;
    
    referee_protocol->game_robot_status.robot_id = 1;
    referee_protocol->game_robot_status.robot_level = 3;
    referee_protocol->game_robot_status.remain_HP = 1000;
    referee_protocol->game_robot_status.max_HP = 1500;
    
    referee_protocol->power_heat_data.chassis_volt = 24000;
    referee_protocol->power_heat_data.chassis_current = 1000;
    referee_protocol->power_heat_data.chassis_power = 24.0;
    referee_protocol->power_heat_data.chassis_power_buffer = 100;
    
    referee_protocol->shoot_data.bullet_type = 1;
    referee_protocol->shoot_data.shooter_id = 1;
    referee_protocol->shoot_data.bullet_freq = 10;
    referee_protocol->shoot_data.bullet_speed = 30.0;

    osDelay(300);
}

// ============================================// 第四节：主循环 - 显示状态
// ============================================
void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    osDelay(500);

    while (true) {
        clear_screen();
        set_cursor(0, 0);

        print("========================================\r\n");
        print("  RefereeCanProtocol Slave Demo         \r\n");
        print("========================================\r\n\r\n");

        print("[Mode]   Automatic Send (Simulated Data)\r\n");
        print("[UART]   huart1 -> Referee System\r\n");
        print("[CAN]    hcan2 -> Host ID: 0x%02X\r\n", HOST_CAN_ID);
        print("[Reg]    0x%02X\r\n\n", CAN_REG_ID);

        print("---------- Sending to Host ----------\r\n");
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

        // 发送数据到Host
        print("Sending data to Host...\r\n");
        bool send_result = referee_protocol->Send(communication::GAME_STATUS);
        if (send_result) {
            print("✓ GAME_STATUS sent\r\n");
        } else {
            print("✗ GAME_STATUS failed\r\n");
        }
        
        send_result = referee_protocol->Send(communication::GAME_ROBOT_STATUS);
        if (send_result) {
            print("✓ GAME_ROBOT_STATUS sent\r\n");
        } else {
            print("✗ GAME_ROBOT_STATUS failed\r\n");
        }
        
        send_result = referee_protocol->Send(communication::POWER_HEAT_DATA);
        if (send_result) {
            print("✓ POWER_HEAT_DATA sent\r\n");
        } else {
            print("✗ POWER_HEAT_DATA failed\r\n");
        }
        
        send_result = referee_protocol->Send(communication::SHOOT_DATA);
        if (send_result) {
            print("✓ SHOOT_DATA sent\r\n");
        } else {
            print("✗ SHOOT_DATA failed\r\n");
        }

        print("[Note] Data sent via CAN Bridge\r\n");

        osDelay(1000);
    }
}
