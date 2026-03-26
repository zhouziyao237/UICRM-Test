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

#include "referee_task.h"

// ---------------------------- Referre Data Code ----------------------------

communication::Referee* Referee_data = nullptr;

// ---------------------------- Referre Main Code ----------------------------

void InitRefPort() {
    // 启动裁判系统
    if (!referee_uart)
        RM_ASSERT_TRUE(false, "Referee init failed!\r\n");
    Referee_data = new communication::Referee(referee_uart);
    SysStartData.RefereeInitFlag = 1;
    osDelay(50);
}

uint8_t RefereeIsOnline() {
    RobotStatus.RefereeOnline = (uint8_t)Referee_data->IsOnline();
    return RobotStatus.RefereeOnline;
}