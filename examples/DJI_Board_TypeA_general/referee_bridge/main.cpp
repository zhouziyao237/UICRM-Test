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
bsp::CAN* can = nullptr;
CanBridge* can_bridge = nullptr;
RefereeCanProtocol* referee_protocol = nullptr;

// 初始化
void RM_RTOS_Init() {
    osDelay(200);
    print_use_uart(&huart8, true, 915200);
    
    // 初始化CAN
    can = new bsp::CAN(&hcan1, true, 8);
    can_bridge = new CanBridge(can, 0x01);  // 本机CAN节点ID为0x01
    
    // 创建RefereeCanProtocol实例
    // 参数1: CAN Bridge指针
    // 参数2: 本机CAN节点ID
    // 参数3: 声明ID（用于过滤，默认0x42）
    referee_protocol = new RefereeCanProtocol(can_bridge, 0x01, 0x42);
    
    // 准备发送的数据
    // 游戏状态数据
    referee_protocol->game_status.game_type = 1;           // 游戏类型
    referee_protocol->game_status.game_progress = 2;       // 游戏进度
    referee_protocol->game_status.stage_remain_time = 180; // 剩余时间（秒）
    
    // 机器人状态数据
    referee_protocol->game_robot_status.robot_id = 1;         // 机器人ID
    referee_protocol->game_robot_status.robot_level = 3;      // 机器人等级
    referee_protocol->game_robot_status.remain_HP = 1000;     // 当前血量
    referee_protocol->game_robot_status.max_HP = 1500;        // 最大血量
    referee_protocol->game_robot_status.shooter_cooling_rate = 10; //  shooter冷却速率
    referee_protocol->game_robot_status.shooter_heat_limit = 100;  // shooter热量上限
    referee_protocol->game_robot_status.chassis_power_limit = 60;   // 底盘功率限制
    
    print("RefereeCanProtocol Example Started\n");
    print("Using callback+thread+switch pattern\n");
    print("Sending referee data via CAN Bridge...\n");
    
    osDelay(300);
}

// 主循环
void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    
    osDelay(500);
    
    while (true) {
        clear_screen();
        set_cursor(0, 0);
        // 发送游戏状态数据
        bool send_result = referee_protocol->Send(GAME_STATUS);
        if (send_result) {
            print("Sent GAME_STATUS data\r\n");
        } else {
            print("Failed to send GAME_STATUS data\r\n");
        }
        
        // 发送机器人状态数据
        send_result = referee_protocol->Send(GAME_ROBOT_STATUS);
        if (send_result) {
            print("Sent GAME_ROBOT_STATUS data\r\n");
        } else {
            print("Failed to send GAME_ROBOT_STATUS data\r\n");
        }
        
        // 显示接收到的数据
        print("Received Game Status: type=%d, progress=%d, time=%d\r\n",
              referee_protocol->game_status.game_type,
              referee_protocol->game_status.game_progress,
              referee_protocol->game_status.stage_remain_time);
        
        print("Received Robot Status: id=%d, level=%d, HP=%d/%d\r\n",
              referee_protocol->game_robot_status.robot_id,
              referee_protocol->game_robot_status.robot_level,
              referee_protocol->game_robot_status.remain_HP,
              referee_protocol->game_robot_status.max_HP);
        
        // 延时1秒
        osDelay(1000);
    }
}