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
// Created by Administrator on 2025/8/19.
//

#include "RefereeProcessor.h"

#include "Device/bsp_Serial/DeviceSerial.h"
#include "Device/bsp_Serial/bsp_Serial.h"
#include "Referee.h"
#include "RefereeParser.h"
#include "crc_check.h"

namespace RMFC {
    namespace RFS {

        bool RefereeProcessor::process(char data, rwstatus_e rwstatus, RefereeMessage& msg,
                                       RefereeResponse& res, serialPort_t* SerialRFS) {
            _RefereeParser.parse(data, msg);

            switch (rwstatus) {
                case Read_Status: {
                    if (msg.c_state == RFS_STATE_RECEIVED) {
                        ReceiveCommand(msg);
                        msg = RefereeMessage();
                        res = RefereeResponse();
                    }
                } break;
                case Write_Status: {
                    sendResponseV1(SerialRFS, res);
                    msg = RefereeMessage();
                    res = RefereeResponse();
                } break;
                default:
                    return false;
            }

            return msg.c_state != RFS_STATE_IDLE;
        }

        RefereeResult_e RefereeProcessor::ReceiveCommand(RefereeMessage& m) {
            switch (m.cmd) {
                case GAME_STATUS: {
                    _Referee.game_status.game_type = m.readU8();
                    _Referee.game_status.game_progress = m.readU8();
                    _Referee.game_status.stage_remain_time = m.readU16();
                    _Referee.game_status.SyncTimeStamp = m.readU64();
                } break;

                case GAME_RESULT: {
                    _Referee.game_result.winner = m.readU8();
                } break;

                case GAME_ROBOT_HP: {
                    _Referee.game_robot_HP.red_1_robot_HP = m.readU16();
                    _Referee.game_robot_HP.red_2_robot_HP = m.readU16();
                    _Referee.game_robot_HP.red_3_robot_HP = m.readU16();
                    _Referee.game_robot_HP.red_4_robot_HP = m.readU16();
                    _Referee.game_robot_HP.red_5_robot_HP = m.readU16();
                    _Referee.game_robot_HP.red_7_robot_HP = m.readU16();
                    _Referee.game_robot_HP.red_base_HP = m.readU16();
                    _Referee.game_robot_HP.red_outpost_HP = m.readU16();

                    _Referee.game_robot_HP.blue_1_robot_HP = m.readU16();
                    _Referee.game_robot_HP.blue_2_robot_HP = m.readU16();
                    _Referee.game_robot_HP.blue_3_robot_HP = m.readU16();
                    _Referee.game_robot_HP.blue_4_robot_HP = m.readU16();
                    _Referee.game_robot_HP.blue_5_robot_HP = m.readU16();
                    _Referee.game_robot_HP.blue_7_robot_HP = m.readU16();
                    _Referee.game_robot_HP.blue_base_HP = m.readU16();
                    _Referee.game_robot_HP.blue_outpost_HP = m.readU16();
                } break;

                case EVENT_DATA: {
                    _Referee.event_data.event_type = m.readU32();
                } break;

                case SUPPLY_PROJECTILE_ACTION: {
                    _Referee.supply_projectile_action.supply_projectile_id = m.readU8();
                    _Referee.supply_projectile_action.supply_robot_id = m.readU8();
                    _Referee.supply_projectile_action.supply_projectile_step = m.readU8();
                    _Referee.supply_projectile_action.supply_projectile_num = m.readU8();
                } break;

                case REFEREE_WARNING: {
                    _Referee.referee_warning.level = m.readU8();
                    _Referee.referee_warning.offending_robot_id = m.readU8();
                } break;

                case DART_REMAINING_TIME: {
                    _Referee.dart_remaining_time.dart_remaining_time = m.readU8();
                    _Referee.dart_remaining_time.dart_info = m.readU16();
                } break;

                case GAME_ROBOT_STATUS: {
                    _Referee.game_robot_status.robot_id = m.readU8();
                    _Referee.game_robot_status.robot_level = m.readU8();
                    _Referee.game_robot_status.remain_HP = m.readU16();
                    _Referee.game_robot_status.max_HP = m.readU16();
                    _Referee.game_robot_status.shooter_cooling_rate = m.readU16();
                    _Referee.game_robot_status.shooter_heat_limit = m.readU16();
                    _Referee.game_robot_status.chassis_power_limit = m.readU16();
                    uint8_t power_output = m.readU8();
                    _Referee.game_robot_status.mains_power_gimbal_output = power_output & 0x01;
                    _Referee.game_robot_status.mains_power_chassis_output =
                        (power_output >> 1) & 0x01;
                    _Referee.game_robot_status.mains_power_shooter_output =
                        (power_output >> 2) & 0x01;
                } break;

                case POWER_HEAT_DATA: {
                    _Referee.power_heat_data.chassis_volt = m.readU16();
                    _Referee.power_heat_data.chassis_current = m.readU16();
                    _Referee.power_heat_data.chassis_power = m.readfloat();
                    _Referee.power_heat_data.chassis_power_buffer = m.readU16();
                    _Referee.power_heat_data.shooter_id1_17mm_cooling_heat = m.readU16();
                    _Referee.power_heat_data.shooter_id2_17mm_cooling_heat = m.readU16();
                    _Referee.power_heat_data.shooter_id1_42mm_cooling_heat = m.readU16();
                } break;

                case GAME_ROBOT_POS: {
                    _Referee.game_robot_pos.x = m.readfloat();
                    _Referee.game_robot_pos.y = m.readfloat();
                    _Referee.game_robot_pos.yaw = m.readfloat();
                } break;

                case BUFF: {
                    _Referee.buff.recovery_buff = m.readU8();
                    _Referee.buff.cooling_buff = m.readU8();
                    _Referee.buff.defence_buff = m.readU8();
                    _Referee.buff.vulnerability_buff = m.readU8();
                    _Referee.buff.attack_buff = m.readU16();
                } break;

                case AERIAL_ROBOT_ENERGY: {
                    _Referee.aerial_robot_energy.airforce_status = m.readU8();
                    _Referee.aerial_robot_energy.time_remain = m.readU8();
                } break;

                case ROBOT_HURT: {
                    uint8_t hurt_data = m.readU8();
                    _Referee.robot_hurt.armor_id = hurt_data & 0x0F;
                    _Referee.robot_hurt.hurt_type = (hurt_data >> 4) & 0x0F;
                } break;

                case SHOOT_DATA: {
                    _Referee.shoot_data.bullet_type = m.readU8();
                    _Referee.shoot_data.shooter_id = m.readU8();
                    _Referee.shoot_data.bullet_freq = m.readU8();
                    _Referee.shoot_data.bullet_speed = m.readfloat();
                } break;

                case BULLET_REMAINING: {
                    _Referee.bullet_remaining.bullet_remaining_num_17mm = m.readU16();
                    _Referee.bullet_remaining.bullet_remaining_num_42mm = m.readU16();
                    _Referee.bullet_remaining.coin_remaining_num = m.readU16();
                } break;

                case RFID_STATUS: {
                    _Referee.rfid_status.rfid_status = m.readU32();
                } break;

                case DART_CLIENT_CMD: {
                    _Referee.dart_client_cmd.dart_launch_opening_status = m.readU8();
                    _Referee.dart_client_cmd.dart_attack_target = m.readU8();
                    _Referee.dart_client_cmd.target_change_time = m.readU16();
                    _Referee.dart_client_cmd.operate_launch_cmd_time = m.readU16();
                } break;

                case REMOTE_CONTROL_DATA: {
                    // 读取鼠标数据
                    for (int i = 0; i < 12; i++) {
                        _Referee.remote_control.mouse_data[i] = m.readU8();
                    }
                    // 读取键盘数据
                    for (int i = 0; i < 10; i++) {
                        _Referee.remote_control.keyboard_data[i] = m.readU8();
                    }
                    _Referee.remote_control.reserved = m.readU16();
                } break;

                default:
                    return MSP_RESULT_CMD_UNKNOWN;
            }
            return MSP_RESULT_ACK;
        }

        RefereeResult_e RefereeProcessor::SendCommand(RefereeMessage& m, RefereeResponse& r) {
            r.cmd = m.cmd;
            r.result = 1;
            // const unsigned int dataSize = m.remain();
            switch (m.cmd) {
                case GAME_STATUS: {
                    r.writeU8(_Referee.game_status.game_type);
                    r.writeU8(_Referee.game_status.game_progress);
                    r.writeU16(_Referee.game_status.stage_remain_time);
                    r.writeU64(_Referee.game_status.SyncTimeStamp);
                } break;

                case GAME_RESULT: {
                    r.writeU8(_Referee.game_result.winner);
                } break;

                case GAME_ROBOT_HP: {
                    r.writeU16(_Referee.game_robot_HP.red_1_robot_HP);
                    r.writeU16(_Referee.game_robot_HP.red_2_robot_HP);
                    r.writeU16(_Referee.game_robot_HP.red_3_robot_HP);
                    r.writeU16(_Referee.game_robot_HP.red_4_robot_HP);
                    r.writeU16(_Referee.game_robot_HP.red_5_robot_HP);
                    r.writeU16(_Referee.game_robot_HP.red_7_robot_HP);
                    r.writeU16(_Referee.game_robot_HP.red_base_HP);
                    r.writeU16(_Referee.game_robot_HP.red_outpost_HP);

                    r.writeU16(_Referee.game_robot_HP.blue_1_robot_HP);
                    r.writeU16(_Referee.game_robot_HP.blue_2_robot_HP);
                    r.writeU16(_Referee.game_robot_HP.blue_3_robot_HP);
                    r.writeU16(_Referee.game_robot_HP.blue_4_robot_HP);
                    r.writeU16(_Referee.game_robot_HP.blue_5_robot_HP);
                    r.writeU16(_Referee.game_robot_HP.blue_7_robot_HP);
                    r.writeU16(_Referee.game_robot_HP.blue_base_HP);
                    r.writeU16(_Referee.game_robot_HP.blue_outpost_HP);
                } break;

                case EVENT_DATA: {
                    r.writeU32(_Referee.event_data.event_type);
                } break;

                case SUPPLY_PROJECTILE_ACTION: {
                    r.writeU8(_Referee.supply_projectile_action.supply_projectile_id);
                    r.writeU8(_Referee.supply_projectile_action.supply_robot_id);
                    r.writeU8(_Referee.supply_projectile_action.supply_projectile_step);
                    r.writeU8(_Referee.supply_projectile_action.supply_projectile_num);
                } break;

                case REFEREE_WARNING: {
                    r.writeU8(_Referee.referee_warning.level);
                    r.writeU8(_Referee.referee_warning.offending_robot_id);
                } break;

                case DART_REMAINING_TIME: {
                    r.writeU8(_Referee.dart_remaining_time.dart_remaining_time);
                    r.writeU16(_Referee.dart_remaining_time.dart_info);
                } break;

                case GAME_ROBOT_STATUS: {
                    r.writeU8(_Referee.game_robot_status.robot_id);
                    r.writeU8(_Referee.game_robot_status.robot_level);
                    r.writeU16(_Referee.game_robot_status.remain_HP);
                    r.writeU16(_Referee.game_robot_status.max_HP);
                    r.writeU16(_Referee.game_robot_status.shooter_cooling_rate);
                    r.writeU16(_Referee.game_robot_status.shooter_heat_limit);
                    r.writeU16(_Referee.game_robot_status.chassis_power_limit);
                    r.writeU8(_Referee.game_robot_status.mains_power_gimbal_output |
                              (_Referee.game_robot_status.mains_power_chassis_output << 1) |
                              (_Referee.game_robot_status.mains_power_shooter_output << 2));
                } break;

                case POWER_HEAT_DATA: {
                    r.writeU16(_Referee.power_heat_data.chassis_volt);
                    r.writeU16(_Referee.power_heat_data.chassis_current);
                    r.writefloat(_Referee.power_heat_data.chassis_power);
                    r.writeU16(_Referee.power_heat_data.chassis_power_buffer);
                    r.writeU16(_Referee.power_heat_data.shooter_id1_17mm_cooling_heat);
                    r.writeU16(_Referee.power_heat_data.shooter_id2_17mm_cooling_heat);
                    r.writeU16(_Referee.power_heat_data.shooter_id1_42mm_cooling_heat);
                } break;

                case GAME_ROBOT_POS: {
                    r.writefloat(_Referee.game_robot_pos.x);
                    r.writefloat(_Referee.game_robot_pos.y);
                    r.writefloat(_Referee.game_robot_pos.yaw);

                } break;

                case BUFF: {
                    r.writeU8(_Referee.buff.recovery_buff);
                    r.writeU8(_Referee.buff.cooling_buff);
                    r.writeU8(_Referee.buff.defence_buff);
                    r.writeU8(_Referee.buff.vulnerability_buff);
                    r.writeU16(_Referee.buff.attack_buff);
                } break;

                case AERIAL_ROBOT_ENERGY: {
                    r.writeU8(_Referee.aerial_robot_energy.airforce_status);
                    r.writeU8(_Referee.aerial_robot_energy.time_remain);
                } break;

                case ROBOT_HURT: {
                    r.writeU8(_Referee.robot_hurt.armor_id | (_Referee.robot_hurt.hurt_type << 4));
                } break;

                case SHOOT_DATA: {
                    r.writeU8(_Referee.shoot_data.bullet_type);
                    r.writeU8(_Referee.shoot_data.shooter_id);
                    r.writeU8(_Referee.shoot_data.bullet_freq);
                    r.writefloat(_Referee.shoot_data.bullet_speed);

                } break;

                case BULLET_REMAINING: {
                    r.writeU16(_Referee.bullet_remaining.bullet_remaining_num_17mm);
                    r.writeU16(_Referee.bullet_remaining.bullet_remaining_num_42mm);
                    r.writeU16(_Referee.bullet_remaining.coin_remaining_num);
                } break;

                case RFID_STATUS: {
                    r.writeU32(_Referee.rfid_status.rfid_status);
                } break;

                case DART_CLIENT_CMD: {
                    r.writeU8(_Referee.dart_client_cmd.dart_launch_opening_status);
                    r.writeU8(_Referee.dart_client_cmd.dart_attack_target);
                    r.writeU16(_Referee.dart_client_cmd.target_change_time);
                    r.writeU16(_Referee.dart_client_cmd.operate_launch_cmd_time);
                } break;

                case REMOTE_CONTROL_DATA: {
                    // 写入鼠标数据
                    for (int i = 0; i < 12; i++) {
                        r.writeU8(_Referee.remote_control.mouse_data[i]);
                    }
                    // 写入键盘数据
                    for (int i = 0; i < 10; i++) {
                        r.writeU8(_Referee.remote_control.keyboard_data[i]);
                    }
                    r.writeU16(_Referee.remote_control.reserved);
                } break;

                default:
                    r.result = 0;
                    return MSP_RESULT_CMD_UNKNOWN;
            }
            return MSP_RESULT_ACK;
        }

        void RefereeProcessor::sendResponseV1(serialPort_t* SerialRFS, RefereeResponse& r) {
            // todo: 帧头长度为7字节 (SOF + 长度(2) + seq + CRC8 + cmd_id(2))
            uint8_t hdr[7] = {0};
            hdr[0] = SOF;
            uint16_t data_length = r.len;
            hdr[1] = (uint8_t)(data_length & 0xFF);
            hdr[2] = (uint8_t)((data_length >> 8) & 0xFF);
            static uint8_t seq = 0;
            hdr[3] = seq++;
            append_crc8_check_sum(hdr, FRAME_HEADER_LEN);
            hdr[5] = (uint8_t)(r.cmd & 0xFF);
            hdr[6] = (uint8_t)((r.cmd >> 8) & 0xFF);
            serialWriteBuf(SerialRFS, hdr, 7);
            uint16_t checksum = 0;
            if (r.len > 0) {
                serialWriteBuf(SerialRFS, r.data, r.len);
                checksum = get_crc16_check_sum(r.data, r.len, checksum);
            }
            checksum = get_crc16_check_sum(&hdr[1], 6, checksum);
            serialWriteBuf(SerialRFS, (uint8_t*)&checksum, sizeof(checksum));
        }
    }  // namespace RFS
}  // namespace RMFC
