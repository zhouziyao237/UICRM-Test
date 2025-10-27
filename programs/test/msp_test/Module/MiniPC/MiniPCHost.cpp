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

#include <cstring>

#include "MiniPCHost.h"
#include "stdint.h"

extern "C" {
#include "config/MiniPC_protocol.h"
#include "config/MiniPC_protocol_data.h"
}

namespace RMFC {
    namespace MiniPC {
        int MiniPCMessage::remain() {
            return received - read;
        }

        void MiniPCMessage::advance(size_t size) {
            read += size;
        }

        uint8_t MiniPCMessage::readU8() {
            return buffer[read++];
        }

        uint16_t MiniPCMessage::readU16() {
            uint16_t ret;
            ret = readU8();
            ret |= readU8() << 8;
            return ret;
        }

        uint32_t MiniPCMessage::readU32() {
            uint32_t ret;
            ret = readU8();
            ret |= readU8() << 8;
            ret |= readU8() << 16;
            ret |= readU8() << 24;
            return ret;
        }

        uint64_t MiniPCMessage::readU64() {
            uint64_t ret = 0;
            for (int i = 8; i-- > 0;)
                ret |= (uint64_t)readU8() << (i * 8);
            return ret;
        }

        float MiniPCMessage::readfloat() {
            union {
                uint32_t u;
                float f;
            } v = {0};
            v.u = (uint32_t)readU8() | (uint32_t)readU8() << 8 | (uint32_t)readU8() << 16 |
                  (uint32_t)readU8() << 24;
            return v.f;
        }

        void MiniPCResponse::writeData(const char* v, int size) {
            while (size-- > 0)
                writeU8(*v++);
        }

        void MiniPCResponse::writeString(const char* v) {
            while (*v)
                writeU8(*v++);
        }

        void MiniPCResponse::writeU8(uint8_t v) {
            data[len++] = v;
        }

        void MiniPCResponse::writeU16(uint16_t v) {
            writeU8(v >> 0);
            writeU8(v >> 8);
        }

        void MiniPCResponse::writeU32(uint32_t v) {
            writeU8(v >> 0);
            writeU8(v >> 8);
            writeU8(v >> 16);
            writeU8(v >> 24);
        }

        void MiniPCResponse::writeU64(uint64_t v) {
            for (int i = 8; i-- > 0;)
                writeU8(v >> (i * 8));
        }

        void MiniPCResponse::writefloat(float v) {
            uint32_t intVal;
            memcpy(&intVal, &v, sizeof(intVal));  // 安全、无 UB
            writeU8((intVal >> 0) & 0xFF);
            writeU8((intVal >> 8) & 0xFF);
            writeU8((intVal >> 16) & 0xFF);
            writeU8((intVal >> 24) & 0xFF);
        }
    }  // namespace MiniPC
}  // namespace RMFC
