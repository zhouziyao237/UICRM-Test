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

#include "chassis_task.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "referee_task.h"
#include "robot_state.h"
#include "shoot_task.h"
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

void InitMiniPC() {
    communication::Protocol_t protocol_t = {
        .uart = nullptr,
        .usb = atlas_minipc_usb,
    };
    minipc = new communication::Host(protocol_t);
    minipc_thread = new bsp::Thread(thread_init);
    minipc_thread->Start();
}

void minipc_task(void* args) {
    UNUSED(args);
    uint16_t i = 0;
    while (true) {
        i++;
        // microsecond
        if (i % 2 == 0) {
            const ImuData_t imu = ImuGetData();
            minipc->gimbal_current_status.current_imu_pitch = imu.pitch;
            minipc->gimbal_current_status.current_imu_yaw = imu.yaw;
            minipc->gimbal_current_status.current_imu_roll = imu.roll;
            const RefereeSnapshot_t referee = RefereeGetSnapshot();
            minipc->gimbal_current_status.robot_id = referee.robot_id;
            minipc->gimbal_current_status.shooter_id = 0;
            minipc->Transmit(communication::GIMBAL_CURRENT_STATUS);
        }
        if (i % 10 == 0) {
            const ChassisTelemetry_t chassis = ChassisGetTelemetry();
            minipc->chassis_current_status.chassis_enabled =
                chassis.output_enabled ? 1 : 0;
            minipc->chassis_current_status.speed_x = chassis.vx;
            minipc->chassis_current_status.speed_y = chassis.vy;
            minipc->chassis_current_status.speed_turn = chassis.vt;
            minipc->Transmit(communication::CHASSIS_CURRENT_STATUS);
        }

        if (i == 1000) {
            // Secend event
            const RefereeSnapshot_t referee = RefereeGetSnapshot();
            minipc->robot_status_upload.robot_id = referee.robot_id;
            minipc->robot_status_upload.vision_reset = 0;
            minipc->robot_status_upload.location_data[0] = 0;
            minipc->robot_status_upload.location_data[1] = 0;
            minipc->robot_status_upload.is_killed = RobotIsKilled() || robot_hp_dead;
            switch (remote_mode) {
                case REMOTE_MODE_FOLLOW:
                    minipc->robot_status_upload.robot_mode = 1;
                    break;
                case REMOTE_MODE_SPIN:
                    minipc->robot_status_upload.robot_mode = 2;
                    break;
                case REMOTE_MODE_MANUAL:
                    minipc->robot_status_upload.robot_mode = 3;
                    break;
                default:
                    minipc->robot_status_upload.robot_mode = 0;
                    break;
            }
            const ShooterTelemetry_t shooter = ShooterGetTelemetry();
            minipc->robot_status_upload.robot_fric_mode =
                shooter.friction_enabled ? 1 : 0;
            switch (shooter.mode) {
                case SHOOT_MODE_SINGLE:
                    minipc->robot_status_upload.robot_shoot_mode = 1;
                    break;
                case SHOOT_MODE_DOUBLE:
                    minipc->robot_status_upload.robot_shoot_mode = 2;
                    break;
                default:
                    minipc->robot_status_upload.robot_shoot_mode = 0;
                    break;
            }
            minipc->robot_status_upload.supercapacitor_enabled = 0;
            minipc->robot_status_upload.robot_module_online = 0xff;
            minipc->robot_status_upload.is_double_gimbal = 0;
            minipc->Transmit(communication::ROBOT_STATUS_UPLOAD);
            i = 0;
        }
        osDelay(1);
    }
}

