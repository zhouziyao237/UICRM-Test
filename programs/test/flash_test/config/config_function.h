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
// Created by Administrator on 2025/8/6.
//

#ifndef CONFIG_FUNCTION_H
#define CONFIG_FUNCTION_H

#include "config.h"

/* - Motor PID - */

void resatMotorPID(MotorPID_t* MotorPID);
void pgResetFn_MotorPid(void* MotorPids, int size);
void resetindexMotorPID(uint8_t index);
const MotorPID_t* getMotorPID(uint8_t index);
void copyMotorPID(uint8_t dstIndex, uint8_t srcIndex);
void setMotorPID(uint8_t index, const MotorPID_t* MotorPids);


#endif //CONFIG_FUNCTION_H
