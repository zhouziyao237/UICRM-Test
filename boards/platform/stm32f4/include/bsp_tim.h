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
// Created by Administrator on 2025/8/31.
//

#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include <functional>
#include <map>

#include "stm32f4xx_hal.h"

namespace bsp {

    typedef void (*TIM_callback_t)(void* args);

    typedef struct {
        IRQn_Type IRQn;
        uint32_t PreemptPriority;
        uint32_t SubPriority;
        uint32_t CounterMode = TIM_COUNTERMODE_UP;
        uint32_t ClockDivision = TIM_CLOCKDIVISION_DIV1;
        uint32_t AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    } InitConfig;

    class Timer {
      public:
        explicit Timer(TIM_HandleTypeDef* htim, InitConfig InitConfig, uint32_t period_us);
        virtual ~Timer();

        void Start_IT();
        void Stop_IT();

        void RegisterCallback(TIM_callback_t callback, void* args);

      protected:
        TIM_HandleTypeDef* htim_;
        InitConfig InitConfig_;
        uint32_t period_us_;
        uint32_t clock_freq_;
        uint32_t freq_;

        /* new TIM callback */
        TIM_callback_t callback_ = [](void* args) { UNUSED(args); };
        void* callback_args_ = nullptr;

      private:
        friend void TimerCallbackWrapper(TIM_HandleTypeDef* htim);

        static std::map<TIM_HandleTypeDef*, Timer*> ptr_map;
        static Timer* FindInstance(TIM_HandleTypeDef* htim);
    };

}  // namespace bsp

#endif