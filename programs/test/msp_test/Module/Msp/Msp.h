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

#ifndef _MSP_H_
#define _MSP_H_

#include <cstring>

#include "main.h"
#include "stdint.h"

extern "C" {
#include "config/msp_protocol.h"
#include "config/msp_protocol_v2_betaflight.h"
#include "config/msp_protocol_v2_common.h"
}

namespace RMFC {
    namespace MSP {
        static const size_t MSP_BUF_SIZE = 192;

        enum MspState {
            MSP_STATE_IDLE,
            MSP_STATE_HEADER_START,

            MSP_STATE_HEADER_M,
            MSP_STATE_HEADER_V1,
            MSP_STATE_PAYLOAD_V1,
            MSP_STATE_CHECKSUM_V1,

            MSP_STATE_HEADER_X,
            MSP_STATE_HEADER_V2,
            MSP_STATE_PAYLOAD_V2,
            MSP_STATE_CHECKSUM_V2,

            MSP_STATE_RECEIVED,
        };

        enum MspType { MSP_TYPE_CMD, MSP_TYPE_REPLY };

        enum MspVersion { MSP_V1, MSP_V2 };

        struct MspHeaderV1 {
            uint8_t size;
            uint8_t cmd;
        } __attribute__((packed));

        struct MspHeaderV2 {
            uint8_t flags;
            uint16_t cmd;
            uint16_t size;
        } __attribute__((packed));

        typedef enum {
            MSP_PENDING_NONE,
            MSP_PENDING_BOOTLOADER_ROM,
            MSP_PENDING_CLI,
            MSP_PENDING_BOOTLOADER_FLASH,
        } mspPendingSystemRequest_e;

        class MspMessage {
          public:
            MspMessage() : c_state(MSP_STATE_IDLE), expected(0), received(0), read(0) {
            }
            MspState c_state;
            MspType dir;
            MspVersion version;
            mspPendingSystemRequest_e pendingRequest;
            uint8_t flags;
            uint16_t cmd;
            uint16_t expected;
            uint16_t received;
            uint16_t read;
            uint8_t checksum;
            uint8_t checksum2;
            uint8_t buffer[MSP_BUF_SIZE];

            int remain();

            void advance(size_t size);

            uint8_t readU8();

            uint16_t readU16();

            uint32_t readU32();
        };

        class MspResponse {
          public:
            MspResponse() : len(0) {
            }
            MspVersion version;
            uint16_t cmd;
            int8_t result;
            uint8_t direction;
            uint16_t len;
            uint8_t data[MSP_BUF_SIZE];

            void writeData(const char* v, int size);

            void writeString(const char* v);

            // void writeString(const char *ifsh);

            void writeU8(uint8_t v);

            void writeU16(uint16_t v);

            void writeU32(uint32_t v);
        };
    }  // namespace Msp
}  // namespace RMFC

#endif
