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

#include "MotorPWMBase.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "buzzer_notes.h"
#include "buzzer_task.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "minipc_task.h"
#include "rc.h"
#include "referee_task.h"
#include "shoot_task.h"
#include "sys.h"

void RM_RTOS_Init(void){
    // 设置高精度定时器以能够获取微秒级别的精度的运行时间数据
    bsp::SetHighresClockTimer(&htim5);
    // 初始化调试模式

    InitRMSys();
    InitRC();

    InitIMU();
    init_buzzer();
    InitReferee();
    init_minipc();
    InitGimbal();
    InitChassis();
    InitShoot();

    HAL_Delay(1000);
}

void RM_RTOS_Threads_Init(void) {
    // 分别启动每个任务
    buzzerTaskHandle = osThreadNew(buzzerTask, nullptr, &buzzerTaskAttribute);
    RCTaskHandle = osThreadNew(RCTask, nullptr, &RCTaskAttribute);
    SysTaskHandle = osThreadNew(SysTask, nullptr, &SysTaskAttribute);
    GimbalTaskHandle = osThreadNew(GimbalTask, nullptr, &GimbalTaskAttribute);
    ChassisTaskHandle = osThreadNew(ChassisTask, nullptr, &ChassisTaskAttribute);
    ShootTaskHandle = osThreadNew(ShootTask, nullptr, &ShootTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);

    osDelay(500);
    while (true) {
        set_cursor(0, 0);
        clear_screen();
        for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
            print("CH%d: %.3f |", i, rcData[i]);
            if (i % 4 == 3)
                print("\r\n");
        }
        print("\r\n");

        bool is_connected = vtm->IsOnline();
        print("VTM Status: %s | Updates: %lu\r\n",
              is_connected ? "CONNECTED" : "DISCONNECTED", vtm->timestamp);

        // print("Chassis Volt: %.3f\r\n", referee->power_heat_data.chassis_volt / 1000.0);
        // print("Chassis Curr: %.3f\r\n", referee->power_heat_data.chassis_current / 1000.0);
        // print("Chassis Power: %.3f\r\n", referee->power_heat_data.chassis_power);
        // print("\r\n");
        // print("Shooter Cooling Heat: %hu\r\n",
        //       referee->power_heat_data.shooter_id1_17mm_cooling_heat);
        // print("Bullet Frequency: %hhu\r\n", referee->shoot_data.bullet_freq);
        // print("Bullet Speed: %.3f\r\n", referee->shoot_data.bullet_speed);
        // print("INS Angle: %.3f %.3f %.3f\r\n", ahrs->INS_angle[0], ahrs->INS_angle[1],
        //       ahrs->INS_angle[2]);
        // print("Vision Target: %.3f %.3f\r\n", minipc->target_angle.target_pitch,
        //       minipc->target_angle.target_yaw);

        print("IMU data:\r\n");
        print("gyro: x:%.2f, y:%.2f, z:%.2f\r\n", bmi088->gyro_[0], bmi088->gyro_[1], bmi088->gyro_[2]);
        print("accel: x:%.2f, y:%.2f, z:%.2f\r\n", bmi088->accel_[0], bmi088->accel_[1],
              bmi088->accel_[2]);
        print("INS Angle: %.3f %.3f %.3f\r\n", ahrs->INS_angle[0], ahrs->INS_angle[1],
              ahrs->INS_angle[2]);

        osDelay(50);
    }
}
