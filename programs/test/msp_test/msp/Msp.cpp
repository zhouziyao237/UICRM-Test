
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