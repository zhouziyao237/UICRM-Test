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

#pragma once
#include "cmsis_os2.h"
#include "main.h"

/**
 * @brief EventThread主循环函数
 *
 * @warning ⚠️【重要警告：不看这个注释吃亏自负系列】⚠️
 *
 * 如果你在主循环或者其他线程里写了死循环却没有调用osDelay或其他类似函数，
 * 那么恭喜你，即将体验到"我的线程为什么不动"的灵异事件！
 *
 * @note 🎯【现象描述】：
 * EventThread创建了但就是不干活，就像你喊"小爱同学"但它在装死一样。
 *
 * @note 🤔【为什么会这样？】：
 * RTOS是个讲先来后到的地方，如果你的主线程是个"霸总"，占着CPU不放，
 * 其他线程就只能在旁边干瞪眼，等着你给一点点时间片。
 *
 * @note 💡【解决方案】：
 * 在你的死循环里加个osDelay(1)或者类似的函数，就像给其他线程发个"你先请"的手势。
 *
 * @note 🛑【特别提醒】：
 * 这类问题造成的bug千奇百怪，调试起来能把人逼疯，
 * 多数情况下都是因为标志位没复位或者线程调度被阻塞导致的。
 *
 * @code
 * // ❌ 错误示例 - 线程饿死型
 * while(1) {
 *     // 干点自己的事
 *     // 但是不给其他线程活路
 * }
 *
 * // ✅ 正确示例 - 和谐共存型
 * while(1) {
 *     // 干点自己的事
 *     osDelay(1);  // 给其他线程一个机会
 * }
 * @endcode
 *
 * @attention 📢 记住：在RTOS的世界里，独乐乐不如众乐乐，资源共享才是王道！
 */

namespace bsp {

    typedef void (*thread_func_t)(void* args);

    typedef struct {
        thread_func_t func;
        void* args;
        osThreadAttr_t attr;
    } thread_init_t;

    class Thread {
      public:
        Thread(thread_init_t init);

        virtual ~Thread();

        virtual void Start();

        void Join();

        void Pause();

        void Resume();

        void Wait(uint32_t millisec = 0, uint32_t signal = 0);

        void Set(uint32_t signal = 0);

        void* GetArgs();

      protected:
        osThreadId_t thread_handle_;
        thread_func_t func_;
        osThreadAttr_t attr_;
        void* args_;
        static const uint32_t rx_signal_ = 1 << 0;
    };

    class EventThread : public Thread {
      public:
        EventThread(thread_init_t init);

        void Start() override final;

      private:
        static void ThreadFunc(void* args);
    };

}  // namespace bsp