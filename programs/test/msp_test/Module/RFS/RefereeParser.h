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

#pragma once

#include "Referee.h"

extern "C" {
#include "config/Referee_protocol.h"
#include "config/Referee_protocol_data.h"
}

namespace RMFC {
    namespace RFS {
        class RefereeParser {
          public:
            RefereeParser() {};

            void parse(const char c, RefereeMessage& msg);
        };
    }  // namespace RFS
}  // namespace RMFC