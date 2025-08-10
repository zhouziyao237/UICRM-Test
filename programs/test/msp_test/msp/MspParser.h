#ifndef _ESPFC_MSP_PARSER_H_
#define _ESPFC_MSP_PARSER_H_

#include "Msp.h"
#include "crc_check.h"

namespace RMFC {
    namespace Msp {
        class MspParser {
          public:
            MspParser() { }

            void parse(char c, MspMessage& msg);
        };
    }  // namespace Msp
}  // namespace Espfc

#endif
