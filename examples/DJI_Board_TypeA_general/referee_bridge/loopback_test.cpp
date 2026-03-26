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

#include "bsp_can.h"
#include "bsp_print.h"
#include "can_bridge.h"
#include "cmsis_os.h"
#include "protocol.h"

using namespace communication;

// 全局变量
bsp::CAN* can1 = nullptr;
bsp::CAN* can2 = nullptr;
CanBridge* can_bridge_tx = nullptr;
CanBridge* can_bridge_rx = nullptr;
RefereeCanProtocol* referee_protocol_tx = nullptr;
RefereeCanProtocol* referee_protocol_rx = nullptr;



// 初始化
void RM_RTOS_Init() {
    osDelay(200);
    print_use_uart(&huart8, true, 915200);

    // 初始化BSP
    can1 = new bsp::CAN(&hcan1, true, 8);  // CAN1 作为发送端
    can2 = new bsp::CAN(&hcan2, true, 8);  // CAN2 作为接收端

    // 创建CAN Bridge实例
    // CAN1 作为发送端，节点ID为0x01
    can_bridge_tx = new CanBridge(can1, 0x01);
    // CAN2 作为接收端，节点ID为0x02
    can_bridge_rx = new CanBridge(can2, 0x02);

    // 创建RefereeCanProtocol实例
    // 发送端
    referee_protocol_tx = new RefereeCanProtocol(can_bridge_tx, 0x01, 0x42);
    // 接收端
    referee_protocol_rx = new RefereeCanProtocol(can_bridge_rx, 0x02, 0x42);

    // 注意：RefereeCanProtocol现在使用callback+thread+switch模式
    // 数据会自动解析到对应的成员变量中，无需注册回调函数

    // 准备发送的数据
    // 游戏状态数据
    referee_protocol_tx->game_status.game_type = 1;           // 游戏类型
    referee_protocol_tx->game_status.game_progress = 2;       // 游戏进度
    referee_protocol_tx->game_status.stage_remain_time = 180; // 剩余时间（秒）

    // 机器人状态数据
    referee_protocol_tx->game_robot_status.robot_id = 1;         // 机器人ID
    referee_protocol_tx->game_robot_status.robot_level = 3;      // 机器人等级
    referee_protocol_tx->game_robot_status.remain_HP = 1000;     // 当前血量
    referee_protocol_tx->game_robot_status.max_HP = 1500;        // 最大血量
    referee_protocol_tx->game_robot_status.shooter_cooling_rate = 10; //  shooter冷却速率
    referee_protocol_tx->game_robot_status.shooter_heat_limit = 100;  // shooter热量上限
    referee_protocol_tx->game_robot_status.chassis_power_limit = 60;   // 底盘功率限制

    print("RefereeCanProtocol Loopback Test Started\r\n");
    print("CAN1 (TX) <--> CAN2 (RX)\r\n");
    print("Sending referee data via CAN Bridge...\r\n");

    osDelay(300);
}

// 主循环
void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    osDelay(500);

    int test_count = 0;
    const int MAX_TESTS = 100;  // 测试100次

    while (test_count < MAX_TESTS) {
        // 发送游戏状态数据
        print("[TX] Sending GAME_STATUS data...\r\n");
        bool send_result = referee_protocol_tx->Send(GAME_STATUS);
        if (send_result) {
            print("[TX] Successfully sent GAME_STATUS data\r\n");
        } else {
            print("[TX] Failed to send GAME_STATUS data\r\n");
        }

        // 延时500ms
        osDelay(500);

        // 发送机器人状态数据
        print("[TX] Sending GAME_ROBOT_STATUS data...\r\n");
        send_result = referee_protocol_tx->Send(GAME_ROBOT_STATUS);
        if (send_result) {
            print("[TX] Successfully sent GAME_ROBOT_STATUS data\r\n");
        } else {
            print("[TX] Failed to send GAME_ROBOT_STATUS data\r\n");
        }

        // 显示接收到的数据
        print("[RX] Received Game Status: type=%d, progress=%d, time=%d\r\n",
              referee_protocol_rx->game_status.game_type,
              referee_protocol_rx->game_status.game_progress,
              referee_protocol_rx->game_status.stage_remain_time);

        print("[RX] Received Robot Status: id=%d, level=%d, HP=%d/%d\r\n",
              referee_protocol_rx->game_robot_status.robot_id,
              referee_protocol_rx->game_robot_status.robot_level,
              referee_protocol_rx->game_robot_status.remain_HP,
              referee_protocol_rx->game_robot_status.max_HP);

        // 延时1秒
        osDelay(1000);

        test_count++;
        print("Test %d/%d completed\r\n\r\n", test_count, MAX_TESTS);
    }

    print("Loopback test completed!\r\n");

    while (true) {
        osDelay(1000);
    }
}