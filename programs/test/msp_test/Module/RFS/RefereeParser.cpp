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

#include "RefereeParser.h"

#include "Referee.h"
#include "crc_check.h"

extern "C" {
#include "config/Referee_protocol.h"
#include "config/Referee_protocol_data.h"
}

namespace RMFC {
    namespace RFS {

        void RefereeParser::parse(const char c, RefereeMessage& msg) {
            switch (msg.c_state) {
                case RFS_STATE_IDLE:  // sync char 1 '0xA5'
                    if ((uint8_t)c == SOF) {
                        msg.c_state = RFS_STATE_HEADER_SOF;
                        msg.received = 0;
                        msg.read = 0;
                        msg.checksum = 0;
                        msg.checksum2 = 0;
                    }
                    break;

                case RFS_STATE_HEADER_SOF:  // SOF
                    msg.buffer[msg.received++] = c;
                    if (msg.received == FRAME_HEADER_LEN) {
                        // check to handle the frame
                        if (verify_crc8_check_sum(msg.buffer, FRAME_HEADER_LEN)) {
                            msg.expected = (msg.buffer[1] << 8) | msg.buffer[2];  // data length
                            msg.seq = msg.buffer[3];                              // sequence number

                            if (msg.expected > RFS_BUF_SIZE) {
                                msg.c_state = RFS_STATE_IDLE;
                            } else {
                                msg.c_state = RFS_STATE_HEADER_CMD_ID;
                                msg.received = 0;
                            }
                        } else {
                            msg.c_state = RFS_STATE_IDLE;
                        }
                    }
                    break;

                case RFS_STATE_HEADER_CMD_ID:
                    msg.buffer[msg.received++] = c;
                    if (msg.received == CMD_ID_LEN) {
                        // 解析命令ID
                        msg.cmd = (msg.buffer[1] << 8) | msg.buffer[0];
                        msg.received = 0;
                        msg.c_state =
                            (msg.expected > 0) ? RFS_STATE_PAYLOAD : RFS_STATE_CHECKSUM_CRC16;
                    }
                    break;

                case RFS_STATE_PAYLOAD:
                    msg.buffer[FRAME_HEADER_LEN + CMD_ID_LEN + msg.received] = c;
                    msg.received++;
                    if (msg.received == msg.expected) {
                        msg.c_state = RFS_STATE_CHECKSUM_CRC16;
                        msg.received = 0;
                    }
                    break;

                case RFS_STATE_CHECKSUM_CRC16:
                    if (msg.received < FRAME_TAIL_LEN) {
                        msg.buffer[FRAME_HEADER_LEN + CMD_ID_LEN + msg.expected + msg.received] = c;
                        msg.received++;

                        if (msg.received == FRAME_TAIL_LEN) {
                            uint8_t* frame = msg.buffer;
                            int frame_len =
                                FRAME_HEADER_LEN + CMD_ID_LEN + msg.expected + FRAME_TAIL_LEN;
                            if (verify_crc16_check_sum(frame, frame_len)) {
                                msg.c_state = RFS_STATE_RECEIVED;
                            } else {
                                msg.c_state = RFS_STATE_IDLE;
                            }
                        }
                    }
                    break;

                default:
                    msg.c_state = RFS_STATE_IDLE;
                    break;
            }
        }

    }  // namespace RFS
}  // namespace RMFC
