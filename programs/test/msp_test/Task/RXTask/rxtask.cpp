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
// Created by Administrator on 2025/8/27.
//

#include "rxtask.h"

#include "platform.h"
#include "target.h"

namespace RMFC {
    namespace RX {

        static rxState_e rxState;

        bool UpdateRxMainInProgress(void) {
            return (rxState != RX_STATE_CHECK);
        }

    }  // namespace RX
}  // namespace RMFC