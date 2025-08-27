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

#include "Failsafe.h"

#include "Device/bsp_PG/bsp_pg.h"
#include "Device/bsp_PG/bsp_pg_ids.h"
#include "Device/config/runtime_config.h"
#include "bsp_os.h"
#include "utils.h"

namespace RMFC {

    PG_REGISTER_WITH_RESET_TEMPLATE(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 1);

    PG_RESET_TEMPLATE(
        failsafeConfig_t, failsafeConfig,
        .failsafe_delay = 15,      // 1.5秒阶段1周期，信号恢复时可重新获得控制权，在丢包模式下空闲
        .failsafe_off_delay = 10,  // 着陆阶段1秒，如果启用直接丢弃(立即停止)失效保护程序
        .failsafe_switch_mode =
            FAILSAFE_SWITCH_MODE_STAGE1,                   // 默认失效保护开关动作与rc链路丢失相同
        .failsafe_procedure = FAILSAFE_PROCEDURE_DROP_IT,  // 默认完整失效保护程序是
        .failsafe_recovery_delay = 10,  // 需要1秒有效rx数据来允许从失效保护程序中恢复
        .failsafe_stick_threshold = 30  // 30%摇杆偏转量以退出GPS救援程序
    );

    const char* const failsafeProcedureNames[FAILSAFE_PROCEDURE_COUNT] = {
        "DROP",
    };

    static failsafeState_t failsafeState;

    void failsafeReset(void) {
        // todo: 获取配置
        failsafeState.RxData.rxDataFailurePeriod =
            failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND;
        if (failsafeState.RxData.rxDataFailurePeriod < PERIOD_RXDATA_RECOVERY) {
            // avoid transients and ensure reliable arming for minimum of PERIOD_RXDATA_RECOVERY
            // (200ms)
            failsafeState.RxData.rxDataFailurePeriod = PERIOD_RXDATA_RECOVERY;
        }
        failsafeState.RxData.rxDataRecoveryPeriod =
            failsafeConfig()->failsafe_recovery_delay * MILLIS_PER_TENTH_SECOND;
        if (failsafeState.RxData.rxDataRecoveryPeriod < PERIOD_RXDATA_RECOVERY) {
            // PERIOD_RXDATA_RECOVERY (200ms) is the minimum allowed RxData recovery time
            failsafeState.RxData.rxDataRecoveryPeriod = PERIOD_RXDATA_RECOVERY;
        }
        failsafeState.RxData.validRxDataReceivedAt = 0;
        failsafeState.RxData.validRxDataFailedAt = 0;
        // failsafeState.throttleLowPeriod = 0;
        // failsafeState.landingShouldBeFinishedAt = 0;
        failsafeState.RxData.receivingRxDataPeriod = 0;
        failsafeState.RxData.receivingRxDataPeriodPreset =
            failsafeState.RxData.rxDataRecoveryPeriod;
        failsafeState.Failsafe.phase = FAILSAFE_IDLE;
        failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
        failsafeState.Failsafe.boxFailsafeSwitchWasOn = false;
    }

    void failsafeInit(void) {
        failsafeState.Failsafe.events = 0;
        failsafeState.Failsafe.monitoring = false;

        return;
    }

    failsafePhase_e failsafePhase(void) {
        return failsafeState.Failsafe.phase;
    }

    bool failsafeIsMonitoring(void) {
        return failsafeState.Failsafe.monitoring;
    }

    bool failsafeIsActive(void)  // 检查失效保护是否激活
    {
        return failsafeState.Failsafe.active;
    }

    void failsafeStartMonitoring(void) {
        failsafeState.Failsafe.monitoring = true;
    }

    bool failsafeIsReceivingRxData(void) {
        return (failsafeState.rxLinkState == FAILSAFE_RXLINK_UP);
    }

    void failsafeOnRxSuspend(uint32_t usSuspendPeriod) {
        UNUSED(usSuspendPeriod);
        // failsafeState.validRxData.validRxDataReceivedAt += (usSuspendPeriod / 1000);    // / 1000
        // to convert micros to millis
    }

    void failsafeOnRxResume(void) {
        failsafeState.RxData.validRxDataReceivedAt =
            bsp::GetHighresTickMicroSec();               // 防止RX链接断开触发，重启rx链接
        failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;  // 在rx链接正常时执行
    }

    void failsafeOnValidDataReceived(void)
    // 进入阶段2
    // 在信号丢失后，接收到数据包或BOXFAILSAFE开关恢复时立即执行
    // 如果BOXFAILSAFE恢复，则rxLinkState立即变为RXLINK_UP
    {
        unsetArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);
        // 接收到好的数据包时立即清除OSD中的RXLOSS，并解除锁定

        failsafeState.RxData.validRxDataReceivedAt = bsp::GetHighresTickMilliSec();

        if (failsafeState.RxData.validRxDataFailedAt == 0) {
            // 初始化后，有时只接收有效数据包
            // 此时不知道信号有效多长时间
            // 将首次接收有效数据的时间赋给最后接收有效数据时间
            failsafeState.RxData.validRxDataFailedAt = failsafeState.RxData.validRxDataReceivedAt;
            setArmingDisabled(ARMING_DISABLED_BST);
            // 初始化后需要在rxDataRecoveryPeriod期间保持有效数据才能解锁
        }

        if (CMP<int32_t>(failsafeState.RxData.validRxDataReceivedAt,
                         failsafeState.RxData.validRxDataFailedAt) >
            static_cast<int32_t>(failsafeState.RxData.receivingRxDataPeriodPreset)) {
            // link被认为'up'，在它'down'之后，直到恢复期结束
            failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;
            unsetArmingDisabled(ARMING_DISABLED_BST);
        }
    }

    void failsafeOnValidDataFailed(void)
    // 从rc.c运行，当数据包丢失超过信号验证期(100ms)，或
    // 立即激活BOXFAILSAFE开关，在阶段1延迟结束后
    // 将rxLinkState设置为RXLINK_DOWN，即not up
    // 如果失效保护配置为直接进入阶段2，则在failsafeUpdateState()中立即模拟
    {
        setArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);
        // 信号丢失100ms后在OSD中设置RXLOSS并阻止解锁

        failsafeState.RxData.validRxDataFailedAt = bsp::GetHighresTickMilliSec();
        if ((CMP<int32_t>(failsafeState.RxData.validRxDataFailedAt,
                          failsafeState.RxData.validRxDataReceivedAt) >
             static_cast<int32_t>(failsafeState.RxData.rxDataFailurePeriod))) {
            // 设置rxLinkState = DOWN以启动阶段2失效保护
            failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
            // 显示RXLOSS并阻止解锁
        }
    }

    void failsafeCheckDataFailurePeriod(void)
    // 直接从调度器运行，每10ms，验证链接
    {
        if (CMP<int32_t>(bsp::GetHighresTickMilliSec(),
                         failsafeState.RxData.validRxDataReceivedAt) >
            (int32_t)failsafeState.RxData.rxDataFailurePeriod) {
            // 在阶段1失效保护期后设置链接DOWN，启动阶段2
            failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
            // 无RX链接时阻止解锁
            setArmingDisabled(ARMING_DISABLED_RX_FAILSAFE);
        }
    }

    uint32_t failsafeFailurePeriodMs(void) {
        return failsafeState.RxData.rxDataFailurePeriod;
    }

}  // namespace RMFC
