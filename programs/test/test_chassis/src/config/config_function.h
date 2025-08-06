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

/* - Motor ID - */

void resetMotorId(MotorId_t* MotorIds);
void pgResetFn_MotorId(void* MotorIds, int size);
void resetindexMotorId(uint8_t index);
const MotorId_t* getMotorId(uint8_t index);
void copyMotorId(uint8_t dstIndex, uint8_t srcIndex);
void setMotorId(uint8_t index, const MotorId_t* MotorIds);

/* - Super Cap - */

void resetSuperCap(SuperCap_t* SuperCaps);
void pgResetFn_SuperCap(void* SuperCaps, int size);
void resetindexSuperCap(uint8_t index);
const SuperCap_t* getSuperCap(uint8_t index);
void copySuperCap(uint8_t dstIndex, uint8_t srcIndex);
void setSuperCap(uint8_t index, const SuperCap_t* SuperCaps);

/* - Can Bridge - */

void resetCanBridge(CanBridge_t* CanBridges);
void pgResetFn_CanBridge(void* CanBridges, int size);
void resetindexCanBridge(uint8_t index);
const CanBridge_t* getCanBridge(uint8_t index);
void copyCanBridge(uint8_t dstIndex, uint8_t srcIndex);
void setCanBridge(uint8_t index, const CanBridge_t* CanBridges);

/* - Remote Dbus - */

void resetRemoteDbus(RemoteDbus_t* RemoteDbuses);
void pgResetFn_RemoteDbus(void* RemoteDbuses, int size);
void resetindexRemoteDbus(uint8_t index);
const RemoteDbus_t* getRemoteDbus(uint8_t index);
void copyRemoteDbus(uint8_t dstIndex, uint8_t srcIndex);
void setRemoteDbus(uint8_t index, const RemoteDbus_t* RemoteDbuses);

/* - Remote Sbus - */

void resetRemoteSbus(RemoteSbus_t* RemoteSbuses);
void pgResetFn_RemoteSbus(void* RemoteSbuses, int size);
void resetindexRemoteSbus(uint8_t index);
const RemoteSbus_t* getRemoteSbus(uint8_t index);
void copyRemoteSbus(uint8_t dstIndex, uint8_t srcIndex);
void setRemoteSbus(uint8_t index, const RemoteSbus_t* RemoteSbuses);

/* - Control Model - */

void resetControlModel(ControlModel_t* ControlModels);
void pgResetFn_ControlModel(void* ControlModels, int size);
void resetindexControlModel(uint8_t index);
const ControlModel_t* getControlModel(uint8_t index);
void copyControlModel(uint8_t dstIndex, uint8_t srcIndex);
void setControlModel(uint8_t index, const ControlModel_t* ControlModels);

#endif //CONFIG_FUNCTION_H
