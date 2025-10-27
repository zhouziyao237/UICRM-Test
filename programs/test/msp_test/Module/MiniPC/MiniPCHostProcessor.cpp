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

#include "Device/bsp_Serial/DeviceSerial.h"
#include "Device/bsp_Serial/UARTProtocol.h"
#include "MiniPCHost.h"
#include "MiniPCHostParser.h"
#include "MiniPCHostProcessor.h"
#include "connection_driver.h"
#include "crc_check.h"

namespace RMFC {
    namespace MiniPC {

        MiniPCProcessor::MiniPCProcessor() {
        }

        bool MiniPCProcessor::ProcessReceive(char data, package_t& package) {
            switch (package.rwstatus) {
                case Read_Status: {
                    Heartbeat();
                    _MiniPCParser.parse(data, *package.msg);
                    if (package.msg->c_state == MiniPC_STATE_RECEIVED) {
                        ReceiveCommand(*package.msg);
                        package.msg = (MiniPCMessage*){};
                        package.res = (MiniPCResponse*){};
                    }
                } break;
                default:
                    return false;
            }

            return package.msg->c_state != MiniPC_STATE_IDLE;
        }

        bool MiniPCProcessor::ProcessTransmit(package_t& package, serialPort_t* SerialMiniPC) {
            switch (package.rwstatus) {
                case Read_Status:
                case Write_Status: {
                    SendCommand(*package.msg, *package.res);
                    sendResponseV1(SerialMiniPC, *package.res);
                    package.length = sizeof(*package.res);
                    package.msg = (MiniPCMessage*){};
                    package.res = (MiniPCResponse*){};
                } break;
                default:
                    return false;
            }

            return package.msg->c_state != MiniPC_STATE_IDLE;
        }

        MiniPCResult_e MiniPCProcessor::ReceiveCommand(MiniPCMessage& m) {
            switch (m.cmd) {
                case PACK:
                    memcpy(&MiniPCdata_->pack, m.buffer, m.expected);
                    break;
                case TARGET_ANGLE:
                    // memset(&target_angle, 0, sizeof(target_angle));
                    memcpy(&MiniPCdata_->target_angle, m.buffer, m.expected);
                    break;
                case NO_TARGET_FLAG:
                    memcpy(&MiniPCdata_->no_target_flag, m.buffer, m.expected);
                    break;
                case SHOOT_CMD:
                    memcpy(&MiniPCdata_->shoot_cmd, m.buffer, m.expected);
                    break;
                case ROBOT_MOVE_SPEED:
                    memcpy(&MiniPCdata_->robot_move, m.buffer, m.expected);
                    break;
                case ROBOT_POWER_HEAT_HP_UPLOAD:
                    memcpy(&MiniPCdata_->robot_power_heat_hp_upload, m.buffer, m.expected);
                    break;
                case GIMBAL_CURRENT_STATUS:
                    memcpy(&MiniPCdata_->gimbal_current_status, m.buffer, m.expected);
                    break;
                case CHASSIS_CURRENT_STATUS:
                    memcpy(&MiniPCdata_->chassis_current_status, m.buffer, m.expected);
                    break;
                case AUTOAIM_ENABLE:
                    memcpy(&MiniPCdata_->autoaim_enable, m.buffer, m.expected);
                    break;
                case ROBOT_STATUS_UPLOAD:
                    memcpy(&MiniPCdata_->robot_status_upload, m.buffer, m.expected);
                    break;

                default:
                    return MiniPC_RESULT_CMD_UNKNOWN;
            }
            return MiniPC_RESULT_ACK;
        }

        MiniPCResult_e MiniPCProcessor::SendCommand(MiniPCMessage& m, MiniPCResponse& r) {
            r.cmd = m.cmd;
            r.result = 1;
            // const unsigned int dataSize = m.remain();
            switch (m.cmd) {
                case PACK:
                    r.len = sizeof(pack_t);
                    memcpy(r.data, &MiniPCdata_->pack, r.len);
                    break;
                case TARGET_ANGLE:
                    r.len = sizeof(target_angle_t);
                    memcpy(r.data, &MiniPCdata_->target_angle, r.len);
                    break;
                case NO_TARGET_FLAG:
                    r.len = sizeof(no_target_flag_t);
                    memcpy(r.data, &MiniPCdata_->no_target_flag, r.len);
                    break;
                case SHOOT_CMD:
                    r.len = sizeof(shoot_cmd_t);
                    memcpy(r.data, &MiniPCdata_->shoot_cmd, r.len);
                    break;
                case ROBOT_MOVE_SPEED:
                    r.len = sizeof(robot_move_t);
                    memcpy(r.data, &MiniPCdata_->robot_move, r.len);
                    break;
                case ROBOT_POWER_HEAT_HP_UPLOAD:
                    r.len = sizeof(robot_power_heat_hp_upload_t);
                    memcpy(r.data, &MiniPCdata_->robot_power_heat_hp_upload, r.len);
                    break;
                case GIMBAL_CURRENT_STATUS:
                    r.len = sizeof(gimbal_current_status_t);
                    memcpy(r.data, &MiniPCdata_->gimbal_current_status, r.len);
                    break;
                case CHASSIS_CURRENT_STATUS:
                    r.len = sizeof(chassis_current_status_t);
                    memcpy(r.data, &MiniPCdata_->chassis_current_status, r.len);
                    break;
                case AUTOAIM_ENABLE:
                    r.len = sizeof(autoaim_enable_t);
                    memcpy(r.data, &MiniPCdata_->autoaim_enable, r.len);
                    break;
                case ROBOT_STATUS_UPLOAD:
                    r.len = sizeof(robot_status_upload_t);
                    memcpy(r.data, &MiniPCdata_->robot_status_upload, r.len);
                    break;
                default:
                    r.result = 0;
                    return MiniPC_RESULT_CMD_UNKNOWN;
            }
            return MiniPC_RESULT_ACK;
        }

        void MiniPCProcessor::sendResponseV1(serialPort_t* SerialMiniPC, MiniPCResponse& r) {
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
            serialWriteBuf(SerialMiniPC, hdr, 7);
            uint16_t checksum = 0;
            if (r.len > 0) {
                serialWriteBuf(SerialMiniPC, r.data, r.len);
                checksum = get_crc16_check_sum(r.data, r.len, checksum);
            }
            checksum = get_crc16_check_sum(&hdr[1], 6, checksum);
            serialWriteBuf(SerialMiniPC, (uint8_t*)&checksum, sizeof(checksum));
        }

// ---------------------------------------------- 分割线 ----------------------------------------------

        MiniPCProtocol::MiniPCProtocol(serialPort_t* UARTProt_, bool startThread)
            : UARTProtocol(UARTProt_, startThread) {
            // 设置100ms离线阈值
            SetThreshold(100);
        }

        MiniPCProtocol::~MiniPCProtocol() {
        }

        void MiniPCProtocol::SetThread() {
        }

        void MiniPCProtocol::Receive(void* args) {
            UNUSED(args);
            package.rwstatus = Read_Status;
            uint32_t len = serialRead(UARTProt_, reinterpret_cast<uint8_t**>(data));
            if (len > 0) {
                ProcessReceive(data, package);
            }
        }

        int MiniPCProtocol::Transmit(int cmd_id) {
            package.rwstatus = Write_Status;
            package.res->cmd = cmd_id;
            ProcessTransmit(package, UARTProt_);
            return package.length;
        }

    }  // namespace MiniPC
}  // namespace RMFC
