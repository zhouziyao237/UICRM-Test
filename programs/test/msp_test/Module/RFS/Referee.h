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

#ifndef RFS_H
#define RFS_H

#include <cstring>

#include "main.h"
#include "stdint.h"

extern "C" {
#include "config/Referee_protocol.h"
#include "config/Referee_protocol_data.h"
}

namespace RMFC {
    namespace RFS {
        static const uint8_t SOF = 0xA5;
        static const int FRAME_HEADER_LEN = 5;
        static const int CMD_ID_LEN = 2;
        static const int FRAME_TAIL_LEN = 2;
        static const int BYTE = 8;

        static const size_t RFS_BUF_SIZE = 100;

        enum RfsState {
            RFS_STATE_IDLE,
            RFS_STATE_HEADER_START,
            RFS_STATE_HEADER_SOF,
            RFS_STATE_HEADER_LENGTH,
            RFS_STATE_HEADER_SEQ,
            RFS_STATE_HEADER_CRC8,
            RFS_STATE_HEADER_CMD_ID,
            RFS_STATE_PAYLOAD,
            RFS_STATE_CHECKSUM_CRC16,
            RFS_STATE_RECEIVED,
        };

        struct Header {
            uint8_t SOF;
            uint16_t data_length;
            uint8_t seq;
            uint8_t CRC8;
        } __attribute__((packed));

        // 裁判系统完整帧头结构
        struct FrameHeader {
            uint8_t SOF;
            uint16_t data_length;
            uint8_t seq;
            uint8_t CRC8;
            uint16_t cmd_id;
        } __attribute__((packed));

        class RefereeMessage {
          public:
            RefereeMessage() : c_state(RFS_STATE_IDLE), expected(0), received(0), read(0) {
            }
            RfsState c_state;
            uint16_t cmd;
            size_t expected;
            uint16_t received;
            uint16_t read;
            uint32_t seq;
            uint8_t checksum;  // CRC8
            uint16_t checksum2; // CRC16
            uint8_t buffer[RFS_BUF_SIZE] = {0};;

            int remain();

            void advance(size_t size);

            uint8_t readU8();

            uint16_t readU16();

            uint32_t readU32();

            uint64_t readU64();

            float readfloat();
        };

        class RefereeResponse {
          public:
            RefereeResponse() : len(0) {
            }
            uint16_t cmd;
            int8_t result;
            uint8_t direction;
            uint16_t len;
            uint8_t data[RFS_BUF_SIZE];

            void writeData(const char* v, int size);

            void writeString(const char* v);

            void writeU8(uint8_t v);

            void writeU16(uint16_t v);

            void writeU32(uint32_t v);

            void writeU64(uint64_t v);

            void writefloat(float v);
        };
    }  // namespace RFS
}  // namespace RMFC

#endif //RFS_H
