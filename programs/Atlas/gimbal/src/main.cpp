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
#include "ui_task.h"
#include "sys.h"

void PrintInitStatus() {
    print("\r\n=== System Init Status ===\r\n");
    print("System:     [%c]\r\n", SysStartData.SystemInitFlag ? 'X' : ' ');
    print("IMU:        [%c]\r\n", SysStartData.ImuInitFlag ? 'X' : ' ');
    print("AHRS:       [%c]\r\n", SysStartData.AHRSInitFlag ? 'X' : ' ');
    print("Referee:    [%c]\r\n", SysStartData.RefereeInitFlag ? 'X' : ' ');
    print("Vision:     [%c]\r\n", SysStartData.VisionInitFlag ? 'X' : ' ');
    print("Gimbal:     [%c]\r\n", SysStartData.GimbalInitFlag ? 'X' : ' ');
    print("Chassis:    [%c]\r\n", SysStartData.ChassisInitFlag ? 'X' : ' ');
    print("Shoot:      [%c]\r\n", SysStartData.ShootInitFlag ? 'X' : ' ');
    print("===========================\r\n");
    
    if (!SysStartData.SystemInitFlag) print("ERROR: System not initialized!\r\n");
    bool is_connected = vtm->IsOnline();
    print("VTM Status: %s | Updates: %lu\r\n",
          is_connected ? "CONNECTED" : "DISCONNECTED", vtm->timestamp);
    if (!SysStartData.ImuInitFlag) print("ERROR: IMU not initialized!\r\n");
    if (!SysStartData.AHRSInitFlag) print("ERROR: AHRS not initialized!\r\n");
    if (!SysStartData.GimbalInitFlag) print("ERROR: Gimbal not initialized!\r\n");
    if (!SysStartData.ChassisInitFlag) print("ERROR: Chassis not initialized!\r\n");
    if (!SysStartData.ShootInitFlag) print("ERROR: Shoot not initialized!\r\n");
}

void RM_RTOS_Init(void){
    bsp::SetHighresClockTimer(&htim5);

    InitRMSys();
    RoboSysInit();

    InitRC();
    InitIMU();
    init_buzzer();
    InitRefPort();
    // init_minipc();
    InitGimbal();
    InitChassis();
    InitShoot();
    InitUI();
    PrintInitStatus();

    SysStartData.SystemInitFlag = 1;

}

void RM_RTOS_Threads_Init(void) {
    // 分别启动每个任务
    buzzerTaskHandle = osThreadNew(buzzerTask, nullptr, &buzzerTaskAttribute);
    RCTaskHandle = osThreadNew(RCTask, nullptr, &RCTaskAttribute);
    SysTaskHandle = osThreadNew(SysTask, nullptr, &SysTaskAttribute);
    GimbalTaskHandle = osThreadNew(GimbalTask, nullptr, &GimbalTaskAttribute);
    ChassisTaskHandle = osThreadNew(ChassisTask, nullptr, &ChassisTaskAttribute);
    ShootTaskHandle = osThreadNew(ShootTask, nullptr, &ShootTaskAttribute);
    if (ENABLE_UI)
        uiTaskHandle = osThreadNew(uiTask, nullptr, &uiTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);

    while (!SysStartData.SystemInitFlag) {
        osDelay(10);
    }

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

        print("RobotMode %d Init %d\r\n", RobotMode, SysStartData.SystemInitFlag);

        print("GimbalMode %d Init %d\r\n", GimbalMode, SysStartData.GimbalInitFlag);
        print("ShootMode %d Init %d\r\n", ShootMode, SysStartData.ShootInitFlag);
        print("ChassisMode %d Init %d\r\n", ChassisMode, SysStartData.ChassisInitFlag);

        print("Chassis Motion - vx: %.2f, vy: %.2f, vz: %.2f\r\n",
              ChassisMotion.vx_set_org, ChassisMotion.vy_set_org, ChassisMotion.offset_yaw);

#ifdef HAS_REFEREE

        print("Robot ID:    %d\r\n", Referee_data->game_robot_status.robot_id);
        print("Robot Level: %d\r\n", Referee_data->game_robot_status.robot_level);
        print("Remain HP:   %d / %d\r\n",
              Referee_data->game_robot_status.remain_HP,
              Referee_data->game_robot_status.max_HP);
        print("Power Limit: %d W\r\n\n", Referee_data->game_robot_status.chassis_power_limit);


        print("Voltage: %d mV\r\n", Referee_data->power_heat_data.chassis_volt);
        print("Current: %d mA\r\n", Referee_data->power_heat_data.chassis_current);
        print("Power:   %.2f W\r\n", Referee_data->power_heat_data.chassis_power);
        print("Buffer:  %d J\r\n\n", Referee_data->power_heat_data.chassis_power_buffer);

#endif

        // print("Vision Target: %.3f %.3f\r\n", minipc->target_angle.target_pitch,
        //       minipc->target_angle.target_yaw);
        print("IMU data:\r\n");
        print("gyro: x:%.2f, y:%.2f, z:%.2f\r\n", bmi088->gyro_[0], bmi088->gyro_[1], bmi088->gyro_[2]);
        print("accel: x:%.2f, y:%.2f, z:%.2f\r\n", bmi088->accel_[0], bmi088->accel_[1],
              bmi088->accel_[2]);
        print("INS Angle: %.3f %.3f %.3f\r\n", ahrs->INS_angle[0], ahrs->INS_angle[1],
              ahrs->INS_angle[2]);
        print("Pitch Motor %s, getPitchByMotor %.4f\r\n", pitch_motor->IsOnline() ? "Online" : "Offline", gimbal->getPitchByMotor());
        print("Yaw Motor %s, getYawByMotor %.4f\r\n "
              "getYaw %.4f, yaw_offset_ %.4f\r\n",
              yaw_motor->IsOnline() ? "Online" : "Offline",
              gimbal->getYawByMotor(),
              yaw_motor->GetOutputShaftTheta(),
              gimbal_param->yaw_offset_);
        print("Chassis Yaw_pid_error %.4f\r\n", ChassisData.yaw_pid_error);
        osDelay(100);
    }
}
