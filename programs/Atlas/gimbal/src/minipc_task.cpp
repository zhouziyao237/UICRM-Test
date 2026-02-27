/*###########################################################
 # Copyright (c) 2024. BNU-HKBU UIC RoboMaster              #
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

#include "minipc_task.h"

#include "bsp_thread.h"
#include "bsp_uart.h"

#include "gimbal_task.h"
#include "protocol.h"
#include "sys.h"

communication::ProtocolAdapterManager *MiniPCPort = nullptr;
communication::Host* minipc = nullptr;

bsp::Thread* minipc_thread = nullptr;
const osThreadAttr_t minipc_thread_attr_ = {
    .name = "MiniPCTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
    .tz_module = 0,
    .reserved = 0
};

void minipc_task(void* args);

const bsp::thread_init_t thread_init = {
    .func = minipc_task,
    .args = nullptr,
    .attr = minipc_thread_attr_,
};

void init_minipc() {
    communication::Protocol_t protocol_t = {
        .uart = nullptr,
        .usb = minipc_usb,
    };
    minipc = new communication::Host(protocol_t);
    minipc_thread = new bsp::Thread(thread_init);
    minipc_thread->Start();
}

void minipc_task(void* args) {
    UNUSED(args);
    // uint16_t i = 0;
    while (true) {


        osDelay(1);
    }
}
