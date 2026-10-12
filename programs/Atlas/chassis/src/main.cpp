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
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "chassis_task.h"
#include "power_task.h"

#define CHASSIS_DEBUG

void RM_RTOS_Init(void) {
    // 初始化串口
    print_use_uart(&huart1, true, 921600);

    init_chassis();
    InitPower();
    init_buzzer();
    osDelay(50);
}

void RM_RTOS_Threads_Init(void) {
    buzzerTaskHandle = osThreadNew(buzzerTask, nullptr, &buzzerTaskAttribute);
    PowerTaskHandle =
        osThreadNew(PowerTask, nullptr, &PowerTaskAttribute);
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    osDelay(100);
    Buzzer_Sing(Mario);
    while (true) {
        clear_screen();
        set_cursor(0, 0);

        print("Chassis Motion - vx: %.2f, vy: %.2f, vz: %.2f\r\n",
            chassis->can_bridge_vx_, chassis->can_bridge_vy_, chassis-> can_bridge_vt_);
        print("Chassis EN %s\r\n", chassis->chassis_enable_ ? "Enable" : "Disable");

        static uint32_t print_tick = 0;
        if (HAL_GetTick() - print_tick > 100) {
            print_tick = HAL_GetTick();
            const char* power_observer =
                chassis->IsLocalBusPowerValid()
                    ? "VI+MODEL"
                    : (chassis->IsRefereePowerSource() &&
                               chassis->GetMeasuredPower() >= 0.0f
                           ? "REF+MODEL"
                           : "MODEL");
            print("Power obs=%s Pl=%.1f budget=%.1f model=%.2f est=%.2f pred=%.2f dP=%.1f after=%.2f ref=%.2f bus=%s/%.2f Z=%.1f Zpred=%.1f corr=%.3f LQR=%.3f hard=%.3f final=%.3f warn=%d trip=%d\r\n",
                  power_observer,
                  chassis->GetPowerLimit(),
                  chassis->GetControlPowerBudget(),
                  chassis->GetRequestedModelPower(),
                  chassis->GetPowerEstimate(),
                  chassis->GetPowerPrediction(),
                  chassis->GetPowerRate(),
                  chassis->GetLimitedPowerEstimate(),
                  chassis->GetMeasuredPower(),
                  chassis->IsLocalBusPowerValid() ? "OK" : "OFF",
                  chassis->GetLocalBusPower(),
                  chassis->GetBuffer(),
                  chassis->GetPredictedBuffer(),
                  chassis->GetModelCorrection(),
                  chassis->GetLQRScale(),
                  chassis->GetHardScale(),
                  chassis->GetCurrentScale(),
                  chassis->GetWarnType(),
                  chassis->IsLocalPowerTripActive() ? 1 : 0);
            print("Power bridge seq=%lu age=%lums stale=%d\r\n",
                  static_cast<unsigned long>(chassis->GetPowerBridgeSequence()),
                  static_cast<unsigned long>(PowerControlGetBridgeAgeMs()),
                  PowerControlIsBridgeStale() ? 1 : 0);
            print("Mixer model=OMNI_LEGACY kinematic_scale=%.3f\r\n",
                  chassis->GetKinematicScale());
            print("Wheel target FL=%.2f FR=%.2f BL=%.2f BR=%.2f\r\n",
                  fl_motor->GetTarget(), fr_motor->GetTarget(),
                  bl_motor->GetTarget(), br_motor->GetTarget());
            print("Wheel omega  FL=%.2f FR=%.2f BL=%.2f BR=%.2f\r\n",
                  fl_motor->GetOmega(), fr_motor->GetOmega(),
                  bl_motor->GetOmega(), br_motor->GetOmega());
        }
        chassisMain();
        osDelay(10);
    }
}
