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

#include "bsp_tim.h"

#include <cstring>

#include "bsp_error_handler.h"
#include "tim.h"
#include "cmsis_os.h"
#include "task.h"

namespace bsp {

    void TimerCallbackWrapper(TIM_HandleTypeDef* htim) {
        UBaseType_t isrflags = taskENTER_CRITICAL_FROM_ISR();

        Timer* timer = Timer::FindInstance(htim);
        if (timer != nullptr) {
            TIM_callback_t callback = timer->callback_;
            void* callback_args = timer->callback_args_;

            taskEXIT_CRITICAL_FROM_ISR(isrflags);

            if (callback != nullptr) {
                callback(callback_args);
            }
        } else {
            taskEXIT_CRITICAL_FROM_ISR(isrflags);
        }
    }

    std::map<TIM_HandleTypeDef*, Timer*> Timer::ptr_map;

    /* get initialized uart_t instance given its huart handle struct */
    Timer* Timer::FindInstance(TIM_HandleTypeDef* htim) {
        const auto it = ptr_map.find(htim);
        if (it == ptr_map.end()) {
            return nullptr;
        }

        return it->second;
    }

    Timer::Timer(TIM_HandleTypeDef* htim, InitTIMConfig InitConfig, uint32_t period_us)
        : htim_(htim), InitConfig_(InitConfig), period_us_(period_us) {
    }

    HAL_StatusTypeDef Timer::TimerInit() {
        htim_->Init.CounterMode = InitConfig_.CounterMode;
        htim_->Init.ClockDivision = InitConfig_.ClockDivision;
        htim_->Init.AutoReloadPreload = InitConfig_.AutoReloadPreload;
        freq_ = (1000000 / period_us_);
        uint32_t auto_reload = freq_ > 0 ? clock_freq_ / freq_ - 1 : 0;
        __HAL_TIM_SET_AUTORELOAD(htim_, auto_reload);
        __HAL_TIM_SET_COUNTER(htim_, 0);

        HAL_StatusTypeDef status = HAL_TIM_Base_Start_IT(htim_);
        RM_ASSERT_FALSE(status, "Htim repeated initialization False");
        ptr_map[htim_] = this;
        if (status != HAL_OK)
            return status;

        HAL_NVIC_SetPriority(InitConfig_.IRQn, InitConfig_.PreemptPriority,
                             InitConfig_.SubPriority);
        HAL_NVIC_EnableIRQ(InitConfig_.IRQn);
        return HAL_OK;
    }

    Timer::~Timer() {
        HAL_TIM_Base_Stop_IT(htim_);
        ptr_map.erase(htim_);
    }

    void Timer::Start_IT() {
        HAL_TIM_Base_Start_IT(htim_);
    }

    void Timer::Stop_IT() {
        HAL_TIM_Base_Stop_IT(htim_);
    }

    void Timer::RegisterCallback(TIM_callback_t callback, void* args) {
        callback_ = callback;
        callback_args_ = args;
    }

}  // namespace bsp

void RM_TIM_IRQHandler(TIM_HandleTypeDef* htim) {
    bsp::TimerCallbackWrapper(htim);
}