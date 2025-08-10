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
// Created by Administrator on 2025/8/9.
//

#include "MspParser.h"

#include "Msp.h"
#include "crc_check.h"

extern "C" {
#include "config/msp_protocol.h"
#include "config/msp_protocol_v2_betaflight.h"
#include "config/msp_protocol_v2_common.h"
}

namespace RMFC {
    namespace Msp {
        void MspParser::parse(char c, MspMessage& msg) {
            switch (msg.state) {
                case MSP_STATE_IDLE:  // sync char 1 '$'
                    if (c == '$')
                        msg.state = MSP_STATE_HEADER_START;
                    break;

                case MSP_STATE_HEADER_START:  // sync char 2 'M'
                    msg.read = 0;
                    msg.received = 0;
                    msg.checksum = 0;
                    msg.checksum2 = 0;
                    if (c == 'M') {
                        msg.version = MSP_V1;
                        msg.state = MSP_STATE_HEADER_M;
                    } else if (c == 'X') {
                        msg.version = MSP_V2;
                        msg.state = MSP_STATE_HEADER_X;
                    } else
                        msg.state = MSP_STATE_IDLE;
                    break;

                case MSP_STATE_HEADER_M:  // type '<','>','!'
                    switch (c) {
                        case '>':
                            msg.dir = MSP_TYPE_REPLY;
                            msg.state = MSP_STATE_HEADER_V1;
                            break;
                        case '<':
                            msg.dir = MSP_TYPE_CMD;
                            msg.state = MSP_STATE_HEADER_V1;
                            break;
                        default:
                            msg.state = MSP_STATE_IDLE;
                    }
                    break;

                case MSP_STATE_HEADER_X:  // type '<','>','!'
                    switch (c) {
                        case '>':
                            msg.dir = MSP_TYPE_REPLY;
                            msg.state = MSP_STATE_HEADER_V2;
                            break;
                        case '<':
                            msg.dir = MSP_TYPE_CMD;
                            msg.state = MSP_STATE_HEADER_V2;
                            break;
                        default:
                            msg.state = MSP_STATE_IDLE;
                    }
                    break;

                case MSP_STATE_HEADER_V1:
                    msg.buffer[msg.received++] = c;
                    msg.checksum = crc8_xor(msg.checksum, c);
                    if (msg.received == sizeof(MspHeaderV1)) {
                        const MspHeaderV1* hdr = reinterpret_cast<MspHeaderV1*>(msg.buffer);
                        if (hdr->size > MSP_BUF_SIZE)
                            msg.state = MSP_STATE_IDLE;
                        else {
                            msg.expected = hdr->size;
                            msg.cmd = hdr->cmd;
                            msg.received = 0;
                            msg.state =
                                msg.expected > 0 ? MSP_STATE_PAYLOAD_V1 : MSP_STATE_CHECKSUM_V1;
                        }
                    }
                    break;

                case MSP_STATE_PAYLOAD_V1:
                    msg.buffer[msg.received++] = c;
                    msg.checksum = crc8_xor(msg.checksum, c);
                    if (msg.received == msg.expected) {
                        msg.state = MSP_STATE_CHECKSUM_V1;
                    }
                    break;

                case MSP_STATE_CHECKSUM_V1:
                    msg.state = msg.checksum == c ? MSP_STATE_RECEIVED : MSP_STATE_IDLE;
                    break;

                case MSP_STATE_HEADER_V2:
                    msg.buffer[msg.received++] = c;
                    msg.checksum2 = crc8_dvb_s2(msg.checksum2, c);
                    if (msg.received == sizeof(MspHeaderV2)) {
                        const MspHeaderV2* hdr = reinterpret_cast<MspHeaderV2*>(msg.buffer);
                        if (hdr->size > MSP_BUF_SIZE)
                            msg.state = MSP_STATE_IDLE;
                        else {
                            msg.flags = hdr->flags;
                            msg.cmd = hdr->cmd;
                            msg.expected = hdr->size;
                            msg.received = 0;
                            msg.state =
                                msg.expected > 0 ? MSP_STATE_PAYLOAD_V2 : MSP_STATE_CHECKSUM_V2;
                        }
                    }
                    break;

                case MSP_STATE_PAYLOAD_V2:
                    msg.buffer[msg.received++] = c;
                    msg.checksum2 = crc8_dvb_s2(msg.checksum2, c);
                    if (msg.received == msg.expected) {
                        msg.state = MSP_STATE_CHECKSUM_V2;
                    }
                    break;

                case MSP_STATE_CHECKSUM_V2:
                    msg.state = msg.checksum2 == c ? MSP_STATE_RECEIVED : MSP_STATE_IDLE;
                    break;

                default:
                    // msg.state = MSP_STATE_IDLE;
                    break;
            }
        }
    }  // namespace Msp
}  // namespace Espfc
