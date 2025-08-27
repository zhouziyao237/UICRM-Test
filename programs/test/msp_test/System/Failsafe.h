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
// Created by Administrator on 2025/8/25.
//

#pragma once

#include <stdint.h>

#include "Device/bsp_PG/bsp_pg.h"

#define FAILSAFE_POWER_ON_DELAY_US (1000 * 1000 * 5)
#define MILLIS_PER_TENTH_SECOND 100
#define MILLIS_PER_SECOND 1000
#define PERIOD_OF_1_SECONDS 1 * MILLIS_PER_SECOND
#define PERIOD_OF_3_SECONDS 3 * MILLIS_PER_SECOND
#define PERIOD_OF_30_SECONDS 30 * MILLIS_PER_SECOND
#define PERIOD_RXDATA_FAILURE 10    // millis
#define PERIOD_RXDATA_RECOVERY 200  // millis

namespace RMFC {

    typedef struct failsafeConfig_s {
        uint8_t failsafe_delay;  // Guard time for failsafe activation after signal lost. 1 step =
                                 // 0.1sec - 1sec in example (10)
        uint8_t failsafe_off_delay;    // Time for Landing before motors stop in 0.1sec. 1 step =
                                       // 0.1sec - 20sec in example (200)
        uint8_t failsafe_switch_mode;  // failsafe switch action is 0: Stage 1, 1: Disarms
                                       // instantly, 2: Stage 2
        uint8_t
            failsafe_procedure;  // selected full failsafe procedure is 0: auto-landing, 1: Drop it
        uint16_t failsafe_recovery_delay;  // Time (in 0.1sec) of valid rx data (min 200ms
                                           // PERIOD_RXDATA_RECOVERY) to allow recovering from
                                           // failsafe procedure
        uint8_t
            failsafe_stick_threshold;  // Stick deflection percentage to exit GPS Rescue procedure
    } failsafeConfig_t;

    PG_DECLARE(failsafeConfig_t, failsafeConfig);

    typedef enum {
        FAILSAFE_IDLE = 0,
        FAILSAFE_RX_LOSS_DETECTED,
        FAILSAFE_LANDING,
        FAILSAFE_LANDED,
        FAILSAFE_RX_LOSS_MONITORING,
        FAILSAFE_RX_LOSS_RECOVERED,
        FAILSAFE_Rsferee_RESCUE
    } failsafePhase_e;

    typedef struct validRxData_s {
        uint32_t rxDataFailurePeriod;          // 接收数据失败判定周期
        uint32_t rxDataRecoveryPeriod;         // 接收数据恢复判定周期
        uint32_t validRxDataReceivedAt;        // 最后一次接收到有效数据的时间
        uint32_t validRxDataFailedAt;          // 最后一次接收数据失败的时间
        uint32_t receivingRxDataPeriod;        // 接收有效数据的必需时长周期
        uint32_t receivingRxDataPeriodPreset;  // 接收有效数据周期的预设值
    } RxData_t;

    typedef struct Failsafe_s {
        int16_t events;               // 失效保护事件计数
        bool monitoring;              // 是否正在监控失效保护状态
        bool active;                  // 失效保护是否处于激活状态
        failsafePhase_e phase;        // 当前失效保护阶段
        bool boxFailsafeSwitchWasOn;  // 失效保护开关是否曾经打开
    } Failsafe_t;

    typedef enum {
        FAILSAFE_PROCEDURE_DROP_IT = 0,  // 直接丢弃(立即停止)失效保护程序
        FAILSAFE_PROCEDURE_COUNT         // 失效保护程序类型计数，必须放在最后
    } failsafeProcedure_e;

    extern const char* const failsafeProcedureNames[FAILSAFE_PROCEDURE_COUNT];

    typedef enum {
        FAILSAFE_SWITCH_MODE_STAGE1 = 0,  // 失效保护开关触发阶段1保护动作
        FAILSAFE_SWITCH_MODE_KILL,        // 失效保护开关触发直接终止动作
        FAILSAFE_SWITCH_MODE_STAGE2       // 失效保护开关触发阶段2保护动作
    } failsafeSwitchMode_e;

    typedef enum { FAILSAFE_RXLINK_DOWN = 0, FAILSAFE_RXLINK_UP } failsafeRxLinkState_e;

    //////////////////////
    /// Failsafe State ///
    //////////////////////

    typedef struct failsafeState_s {
        Failsafe_t Failsafe;
        RxData_t RxData;
        failsafeRxLinkState_e rxLinkState;
        // todo: RM比赛中需要添加以下变量, 空中机器人例外, 后面再管
        // uint32_t throttleLowPeriod;             // 油门杆必须保持低于'min_check'的时长
        // uint32_t landingShouldBeFinishedAt;     // 着陆应该完成的时间点
    } failsafeState_t;

    void failsafeInit(void);
    void failsafeReset(void);
    void failsafeUpdateState(void);

    void failsafeStartMonitoring(void);
    void failsafeCheckDataFailurePeriod(void);

    failsafePhase_e failsafePhase(void);
    bool failsafeIsMonitoring(void);
    bool failsafeIsActive(void);
    bool failsafeIsReceivingRxData(void);
    void failsafeOnRxSuspend(uint32_t suspendPeriod);
    void failsafeOnRxResume(void);
    void failsafeOnValidDataReceived(void);
    void failsafeOnValidDataFailed(void);

}  // namespace RMFC
