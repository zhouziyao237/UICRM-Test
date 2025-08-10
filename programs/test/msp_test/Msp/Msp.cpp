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

#include "Msp.h"

#include <cstring>

#include "stdint.h"

extern "C" {
#include "config/msp_protocol.h"
#include "config/msp_protocol_v2_betaflight.h"
#include "config/msp_protocol_v2_common.h"
}
namespace RMFC {

    namespace Msp {
        int MspMessage::remain() {
            return received - read;
        }

        void MspMessage::advance(size_t size) {
            read += size;
        }

        uint8_t MspMessage::readU8() {
            return buffer[read++];
        }

        uint16_t MspMessage::readU16() {
            uint16_t ret;
            ret = readU8();
            ret |= readU8() << 8;
            return ret;
        }

        uint32_t MspMessage::readU32() {
            uint32_t ret;
            ret = readU8();
            ret |= readU8() << 8;
            ret |= readU8() << 16;
            ret |= readU8() << 24;
            return ret;
        };  // namespace Msp

        void MspResponse::writeData(const char* v, int size) {
            while (size-- > 0)
                writeU8(*v++);
        }

        void MspResponse::writeString(const char* v) {
            while (*v)
                writeU8(*v++);
        }

        // void MspResponse::writeString(const char *ifsh)
        // {
        //   const char * p = reinterpret_cast<const char *>(ifsh);
        //   while(true)
        //   {
        //     uint8_t c = static_cast<uint8_t>(*p++);
        //     if (c == 0) break;
        //     writeU8(c);
        //   }
        // }

        void MspResponse::writeU8(uint8_t v) {
            data[len++] = v;
        }

        void MspResponse::writeU16(uint16_t v) {
            writeU8(v >> 0);
            writeU8(v >> 8);
        }

        void MspResponse::writeU32(uint32_t v) {
            writeU8(v >> 0);
            writeU8(v >> 8);
            writeU8(v >> 16);
            writeU8(v >> 24);
        }
    }  // namespace Msp
}  // namespace Espfc