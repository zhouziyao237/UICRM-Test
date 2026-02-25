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

#include "vtm.h"

#include <cmath>
#include <cstring>

#include "bsp_error_handler.h"
#include "cmsis_os2.h"
#include "crc_check.h"
#include "vtm_package.h"
#include "bsp_thread.h"

namespace remote {

    VTM::VTM(bsp::UART* uart) {
        if (uart == nullptr)
            return;
        uart_ = uart;
        uart_->RegisterCallback(nullptr, nullptr);

        bsp::thread_init_t thread_init = {
            .func = VTMTask,
            .args = this,
            .attr = thread_attr_,
        };
        thread_ = new bsp::Thread(thread_init);
        thread_->Start();
    }

    VTM::~VTM() {
        delete thread_;
    }

    void VTM::VTMTask(void* arg) {
        VTM* self = static_cast<VTM*>(arg);
        while (true) {
            self->Heartbeat();
            self->vtm_start_flag = false;
            package_t package = {};
            package.length = self->uart_->Read<true>(&package.data);
            memcpy(self->bufferRx, package.data, package.length);
            int start_idx;
            int end_idx = 0;
            while (end_idx < package.length) {
                start_idx = end_idx;
                while (++end_idx < package.length && self->bufferRx[end_idx] != VTM_FRAME_HEADER_1)
                    ;
                if (end_idx - start_idx >= PACKET_SIZE) {
                    self->VTMupdata(self->bufferRx + start_idx);
                }
            }
            self->vtm_start_flag = true;
            memset(self->bufferRx, 0, package.length);
        }
    }

    void VTM::VTMupdata(uint8_t* data) {
        // 重新解释数据缓冲区并解码到类属性
        vtm_raw_t* repr = reinterpret_cast<vtm_raw_t*>(data);

        if (repr->header1 != VTM_FRAME_HEADER_1 || repr->header2 != VTM_FRAME_HEADER_2)
            return;

        // 参考DBUS的方式，简化处理逻辑
        this->ch0 = (int16_t)(repr->ch0 - VTM_ROCKER_MID);
        this->ch1 = (int16_t)(repr->ch1 - VTM_ROCKER_MID);
        this->ch2 = (int16_t)(repr->ch2 - VTM_ROCKER_MID);
        this->ch3 = (int16_t)(repr->ch3 - VTM_ROCKER_MID);
        this->ch4 = (int16_t)(repr->ch4 - VTM_ROCKER_MID);

        // 死区处理 - 直接简化写法
        this->ch0 = abs(this->ch0) <= VTM_ROCKER_ZERO_DRIFT ? 0 : this->ch0;
        this->ch1 = abs(this->ch1) <= VTM_ROCKER_ZERO_DRIFT ? 0 : this->ch1;
        this->ch2 = abs(this->ch2) <= VTM_ROCKER_ZERO_DRIFT ? 0 : this->ch2;
        this->ch3 = abs(this->ch3) <= VTM_ROCKER_ZERO_DRIFT ? 0 : this->ch3;
        this->ch4 = abs(this->ch4) <= VTM_ROCKER_ZERO_DRIFT ? 0 : this->ch4;

        // 开关和按键 - 简化处理
        this->gear_switch = (vtm_gear_t)repr->gear_switch;
        this->pause_button = repr->pause_button != 0;
        this->custom_left = repr->custom_left != 0;
        this->custom_right = repr->custom_right != 0;

        // 额外通道
        this->ch5 = (int16_t)repr->ch5;

        // 直接内存拷贝 - 参考DBUS方式
        memcpy(&this->mouse, &repr->mouse, sizeof(vtm_mouse_t));
        memcpy(&this->keyboard, &repr->keyboard, sizeof(vtm_keyboard_t));

        this->timestamp = GetLastUptime();
    }

} /* namespace remote */
