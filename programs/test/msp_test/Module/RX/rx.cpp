/*
 * Betaflight / Cleanflight Derivative Work
 * ----------------------------------------
 * Original code: Copyright (c) 2011-2025 Betaflight & Cleanflight contributors
 * Derivative work: Copyright (c) 2025 BNU-HKBU UIC RoboMaster
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

//
// Created by Administrator on 2025/8/22.
//

#include "pg/rx.h"

#include <stdint.h>
#include <string.h>

#include "Device/bsp_PG/bsp_pg.h"
#include "Device/bsp_PG/bsp_pg_ids.h"
#include "Device/config/feature.h"
#include "Module/sbus/sbus.h"
#include "Module/dbus/dbus.h"
#include "System/Failsafe.h"
#include "Task/RXTask/rxtask.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "filter.h"
#include "msp.h"
#include "platform.h"
#include "rx.h"
#include "rx_config.h"
#include "rx_mode.h"
#include "target.h"
#include "utils.h"

namespace RMFC {
    namespace RX {
        const char rcChannelLetters[] = "AERT12345678abcdefgh";

        rxfail_handlers_t rxfail_handlers = {.roll_handler = NULL,
                                             .pitch_handler = NULL,
                                             .yaw_handler = NULL,
                                             .throttle_handler = NULL};

        static uint16_t rssi = 0;     // 范围: [0;1023]
        static uint16_t rssiRaw = 0;  // 范围: [0;1023]
        static timeUs_t lastRssiSmoothingUs = 0;
#ifdef USE_RX_RSSI_DBM
        static int16_t rssiDbm = CRSF_RSSI_MIN;     // 范围: [-130,0]
        static int16_t rssiDbmRaw = CRSF_RSSI_MIN;  // 范围: [-130,0]
#endif                                              // USE_RX_RSSI_DBM
#ifdef USE_RX_RSNR
        static int16_t rsnr = CRSF_SNR_MIN;     // 范围: [-30,20]
        static int16_t rsnrRaw = CRSF_SNR_MIN;  // 范围: [-30,20]
#endif                                          // USE_RX_RSNR
        static timeUs_t lastMspRssiUpdateUs = 0;

        static PT1Filter_t frameErrFilter;
        static PT1Filter_t rssiFilter;
#ifdef USE_RX_RSSI_DBM
        static PT1Filter_t rssiDbmFilter;
#endif  // USE_RX_RSSI_DBM
#ifdef USE_RX_RSNR
        static PT1Filter_t rsnrFilter;
#endif  // USE_RX_RSNR

#ifdef USE_RX_LINK_QUALITY_INFO
        static uint16_t linkQuality = 0;
        static uint8_t rfMode = 0;
#endif

#ifdef USE_RX_LINK_UPLINK_POWER
        static uint16_t uplinkTxPwrMw = 0;  // 上行链路发射功率，单位为毫瓦
#endif

        rssiSource_e rssiSource;
        linkQualitySource_e linkQualitySource;

        static bool rxDataProcessingRequired = false;
        static bool auxiliaryProcessingRequired = false;

        static bool rxSignalReceived = false;
        static bool rxFlightChannelsValid = false;
        static uint8_t rxChannelCount;

        static timeUs_t needRxSignalBefore = 0;
        static timeUs_t suspendRxSignalUntil = 0;
        static uint8_t skipRxSamples = 0;

        static float rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];  // todo: 最后接收到的原始值
        float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];        // todo: 缩放、修改、检查和约束后的值
        uint32_t validRxSignalTimeout[MAX_SUPPORTED_RC_CHANNEL_COUNT];

        rxRuntimeState_t rxRuntimeState;
        static uint8_t rcSampleIndex = 0;

        PG_REGISTER_ARRAY_WITH_RESET_FN(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT,
                                        rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);

        void pgResetFn_rxChannelRangeConfigs(rxChannelRangeConfig_t* rxChannelRangeConfigs,
                                             int size) {
            UNUSED(size);
            // 设置默认校准为全范围和1:1映射
            for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
                rxChannelRangeConfigs[i].min = PWM_RANGE_MIN;
                rxChannelRangeConfigs[i].max = PWM_RANGE_MAX;
            }
        }

        PG_REGISTER_ARRAY_WITH_RESET_FN(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT,
                                        rxFailsafeChannelConfigs, PG_RX_FAILSAFE_CHANNEL_CONFIG, 0);
        void pgResetFn_rxFailsafeChannelConfigs(rxFailsafeChannelConfig_t* rxFailsafeChannelConfigs,
                                                int size) {
            UNUSED(size);
            for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
                rxFailsafeChannelConfigs[i].mode =
                    (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
                rxFailsafeChannelConfigs[i].step = (i == THROTTLE)
                                                       ? CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MIN_USEC)
                                                       : CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MID_USEC);
            }
        }

        void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t* rxChannelRangeConfig) {
            // 设置默认校准为全范围和1:1映射
            for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
                rxChannelRangeConfig->min = PWM_RANGE_MIN;
                rxChannelRangeConfig->max = PWM_RANGE_MAX;
                rxChannelRangeConfig++;
            }
        }

        // --------------------------------------- 分割线 ---------------------------------------

        static float nullReadRawRC(const rxRuntimeState_t* rxRuntimeState, uint8_t channel) {
            UNUSED(rxRuntimeState);
            UNUSED(channel);

            return PPM_RCVR_TIMEOUT;
        }

        static uint8_t nullFrameStatus(rxRuntimeState_t* rxRuntimeState) {
            UNUSED(rxRuntimeState);

            return RX_FRAME_PENDING;
        }

        static bool nullProcessFrame(const rxRuntimeState_t* rxRuntimeState) {
            UNUSED(rxRuntimeState);

            return true;
        }

        STATIC_UNIT_TESTED bool isPulseValid(uint16_t pulseDuration) {
            // if (pulseDuration >= 1050 && pulseDuration <= 1900) {
            if (pulseDuration >= (uint16_t)(rxConfig()->rx_min_usec) &&
                pulseDuration <= (uint16_t)(rxConfig()->rx_max_usec)) {
                return true;
            }
            return false;
        }

#ifdef USE_SERIALRX
        static bool serialRxInit(const rxConfig_t* rxConfig, rxRuntimeState_t* rxRuntimeState) {
            bool enabled = false;
            switch (rxRuntimeState->serialrxProvider) {
#ifdef USE_SERIALRX_SRXL2
                case SERIALRX_SRXL2:
                    enabled = srxl2RxInit(rxConfig, rxRuntimeState);
                    break;
#endif
#ifdef USE_SERIALRX_SBUS
                case SERIALRX_SBUS:
                    enabled = sbusInit(rxConfig, rxRuntimeState);
                    break;
#endif
#ifdef USE_SERIALRX_DBUS
                case SERIALRX_DBUS:
                    enabled = dbusInit(rxConfig, rxRuntimeState);
                    break;
#endif
#ifdef USE_SERIALRX_SUMD
                case SERIALRX_SUMD:
                    enabled = sumdInit(rxConfig, rxRuntimeState);
                    break;
#endif
#ifdef USE_SERIALRX_SUMH
                case SERIALRX_SUMH:
                    enabled = sumhInit(rxConfig, rxRuntimeState);
                    break;
#endif
#ifdef USE_SERIALRX_XBUS
                case SERIALRX_XBUS_MODE_B:
                case SERIALRX_XBUS_MODE_B_RJ01:
                    enabled = xBusInit(rxConfig, rxRuntimeState);
                    break;
#endif
#ifdef USE_SERIALRX_IBUS
                case SERIALRX_IBUS:
                    enabled = ibusInit(rxConfig, rxRuntimeState);
                    break;
#endif
#ifdef USE_SERIALRX_JETIEXBUS
                case SERIALRX_JETIEXBUS:
                    enabled = jetiExBusInit(rxConfig, rxRuntimeState);
                    break;
#endif
#ifdef USE_SERIALRX_CRSF
                case SERIALRX_CRSF:
                    enabled = crsfRxInit(rxConfig, rxRuntimeState);
                    break;
#endif
#ifdef USE_SERIALRX_GHST
                case SERIALRX_GHST:
                    enabled = ghstRxInit(rxConfig, rxRuntimeState);
                    break;
#endif
#ifdef USE_SERIALRX_TARGET_CUSTOM
                case SERIALRX_TARGET_CUSTOM:
                    enabled = targetCustomSerialRxInit(rxConfig, rxRuntimeState);
                    break;
#endif
#ifdef USE_SERIALRX_FPORT
                case SERIALRX_FPORT:
                    enabled = fportRxInit(rxConfig, rxRuntimeState);
                    break;
#endif
                default:
                    enabled = false;
                    break;
            }
            return enabled;
        }
#endif

        void rxInit(void) {
            if (featureIsEnabled(FEATURE_RX_PARALLEL_PWM)) {
                rxRuntimeState.rxProvider = RX_PROVIDER_PARALLEL_PWM;
            } else if (featureIsEnabled(FEATURE_RX_PPM)) {
                rxRuntimeState.rxProvider = RX_PROVIDER_PPM;
            } else if (featureIsEnabled(FEATURE_RX_SERIAL)) {
                rxRuntimeState.rxProvider = RX_PROVIDER_SERIAL;
            } else if (featureIsEnabled(FEATURE_RX_MSP)) {
                rxRuntimeState.rxProvider = RX_PROVIDER_MSP;
            } else if (featureIsEnabled(FEATURE_RX_SPI)) {
                rxRuntimeState.rxProvider = RX_PROVIDER_SPI;
            } else {
                rxRuntimeState.rxProvider = RX_PROVIDER_NONE;
            }
            rxRuntimeState.serialrxProvider =
                static_cast<SerialRXType>(rxConfig()->serialrx_provider);
            rxRuntimeState.rcReadRawFn = nullReadRawRC;
            rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
            rxRuntimeState.rcProcessFrameFn = nullProcessFrame;
            rxRuntimeState.lastRcFrameTimeUs = 0;
            rcSampleIndex = 0;

            for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
                rcData[i] = rxConfig()->midrc;
                validRxSignalTimeout[i] = bsp::GetHighresTickMilliSec() + MAX_INVALID_PULSE_TIME_MS;
            }

            // todo: 简化油门初始化, 不考虑3D模式, 直接初始化为 midrc
            // rcData[THROTTLE] =
            //     (featureIsEnabled(FEATURE_3D)) ? rxConfig()->midrc : rxConfig()->rx_min_usec;

            // 当定义了通过开关解锁时，将ARM开关初始化为OFF位置
            // TODO - 移至 rc_mode.c
            for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
                const modeActivationCondition_t* modeActivationCondition =
                    modeActivationConditions(i);
                if (modeActivationCondition->modeId == BOXARM &&
                    IS_RANGE_USABLE(&modeActivationCondition->range)) {
                    // ARM开关已定义，确定一个OFF值
                    uint16_t value;
                    if (modeActivationCondition->range.startStep > 0) {
                        value = MODE_STEP_TO_CHANNEL_VALUE(
                            (modeActivationCondition->range.startStep - 1));
                    } else {
                        value = MODE_STEP_TO_CHANNEL_VALUE(
                            (modeActivationCondition->range.endStep + 1));
                    }
                    // 将ARM辅助通道初始化为OFF值
                    rcData[modeActivationCondition->auxChannelIndex + NON_AUX_CHANNEL_COUNT] =
                        value;
                }
            }

            switch (rxRuntimeState.rxProvider) {
                default:
#ifdef DEBUG_RX
                    print("RX Provider: Default\r\n");
#endif
                    break;
#ifdef USE_SERIALRX
                case RX_PROVIDER_SERIAL: {
#ifdef DEBUG_RX
                    print("RX Provider: Serial\r\n");
#endif
                    const bool enabled = serialRxInit(rxConfig(), &rxRuntimeState);
#ifdef DEBUG_RX
                    print("Serial RX Init: %s\r\n", enabled ? "Success" : "Failed");
#endif
                    if (!enabled) {
                        rxRuntimeState.rcReadRawFn = nullReadRawRC;
                        rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
                    }
                }

                break;
#endif

#ifdef USE_RX_MSP
                case RX_PROVIDER_MSP:
                    print("RX Provider: MSP\r\n");
                    rxMspInit(rxConfig(), &rxRuntimeState);

                    break;
#endif

#ifdef USE_RX_SPI
                case RX_PROVIDER_SPI: {
                    print("RX Provider: SPI\r\n");
                    const bool enabled = rxSpiInit(rxSpiConfig(), &rxRuntimeState);
                    if (!enabled) {
                        rxRuntimeState.rcReadRawFn = nullReadRawRC;
                        rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
                    }
                }

                break;
#endif

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
                case RX_PROVIDER_PPM:
                    print("RX Provider: PPM\r\n");
                case RX_PROVIDER_PARALLEL_PWM:
                    print("RX Provider: PWM\r\n");
                    rxPwmInit(rxConfig(), &rxRuntimeState);

                    break;
#endif
            }

#if defined(USE_ADC)
            if (featureIsEnabled(FEATURE_RSSI_ADC)) {
                rssiSource = RSSI_SOURCE_ADC;
            } else
#endif
                if (rxConfig()->rssi_channel > 0) {
                rssiSource = RSSI_SOURCE_RX_CHANNEL;
            }

            // 设置源帧RSSI滤波，每FRAME_ERR_RESAMPLE_US取平均值
            PT1FilterInit(
                &frameErrFilter,
                PT1FilterGain(GET_FRAME_ERR_LPF_FREQUENCY(rxConfig()->rssi_src_frame_lpf_period),
                              FRAME_ERR_RESAMPLE_US / 1000000.0));

            // 可配置的滤波量以消除OSD上值的过度跳变
            float k = (256.0f - rxConfig()->rssi_smoothing) / 256.0f;

            PT1FilterInit(&rssiFilter, k);

#ifdef USE_RX_RSSI_DBM
            PT1FilterInit(&rssiDbmFilter, k);
#endif  // USE_RX_RSSI_DBM

#ifdef USE_RX_RSNR
            PT1FilterInit(&rsnrFilter, k);
#endif  // USE_RX_RSNR

            rxChannelCount = min<uint8_t>(rxConfig()->max_aux_channel + NON_AUX_CHANNEL_COUNT,
                                          rxRuntimeState.channelCount);

            print("RX Init Complete\r\n");
        }

        bool rxIsReceivingSignal(void) {
            return rxSignalReceived;
        }

        bool rxAreFlightChannelsValid(void) {
            return rxFlightChannelsValid;
        }

        void suspendRxSignal(void) {
#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
            if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM ||
                rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
                suspendRxSignalUntil = bsp::GetHighresTickMicroSec() + DELAY_1500_MS;  // 1.5秒
                skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
            }
#endif
            failsafeOnRxSuspend(DELAY_1500_MS);  // 1.5秒
        }

        void resumeRxSignal(void) {
#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
            if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM ||
                rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
                suspendRxSignalUntil = bsp::GetHighresTickMicroSec();
                skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
            }
#endif
            failsafeOnRxResume();
        }

#ifdef USE_RX_LINK_QUALITY_INFO
#define LINK_QUALITY_SAMPLE_COUNT 16

        STATIC_UNIT_TESTED uint16_t updateLinkQualitySamples(uint16_t value) {
            static uint16_t samples[LINK_QUALITY_SAMPLE_COUNT];
            static uint8_t sampleIndex = 0;
            static uint16_t sum = 0;

            sum += value - samples[sampleIndex];
            samples[sampleIndex] = value;
            sampleIndex = (sampleIndex + 1) % LINK_QUALITY_SAMPLE_COUNT;
            return sum / LINK_QUALITY_SAMPLE_COUNT;
        }

        void rxSetRfMode(uint8_t rfModeValue) {
            rfMode = rfModeValue;
        }
#endif

        static void setLinkQuality(bool validFrame, timeDelta_t currentDeltaTimeUs) {
            static uint16_t rssiSum = 0;
            static uint16_t rssiCount = 0;
            static timeDelta_t resampleTimeUs = 0;

#ifdef USE_RX_LINK_QUALITY_INFO
            if (linkQualitySource == LQ_SOURCE_NONE) {
                // 计算新的采样均值
                linkQuality = updateLinkQualitySamples(validFrame ? LINK_QUALITY_MAX_VALUE : 0);
            }
#endif

            if (rssiSource == RSSI_SOURCE_FRAME_ERRORS) {
                resampleTimeUs += currentDeltaTimeUs;
                rssiSum += validFrame ? RSSI_MAX_VALUE : 0;
                rssiCount++;

                if (resampleTimeUs >= FRAME_ERR_RESAMPLE_US) {
                    setRssi(rssiSum / rssiCount, rssiSource);
                    rssiSum = 0;
                    rssiCount = 0;
                    resampleTimeUs -= FRAME_ERR_RESAMPLE_US;
                }
            }
        }

        void setLinkQualityDirect(uint16_t linkqualityValue) {
#ifdef USE_RX_LINK_QUALITY_INFO
            linkQuality = linkqualityValue;
#else
            UNUSED(linkqualityValue);
#endif
        }

#ifdef USE_RX_LINK_UPLINK_POWER
        void rxSetUplinkTxPwrMw(uint16_t uplinkTxPwrMwValue) {
            uplinkTxPwrMw = uplinkTxPwrMwValue;
        }
#endif

        bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs) {
            UNUSED(currentTimeUs);
            UNUSED(currentDeltaTimeUs);

            return UpdateRxMainInProgress() || rxDataProcessingRequired ||
                   auxiliaryProcessingRequired;
        }

        FAST_CODE_NOINLINE void rxFrameCheck(timeUs_t currentTimeUs,
                                             timeDelta_t currentDeltaTimeUs) {
            bool signalReceived = false;
            bool useDataDrivenProcessing = true;
            timeDelta_t needRxSignalMaxDelayUs = DELAY_100_MS;

            // DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 2, min<uint16_t>(2000, currentDeltaTimeUs / 100));

            if (UpdateRxMainInProgress()) {
                // 数据包正在处理中，无需检查新数据
                return;
            }

            switch (rxRuntimeState.rxProvider) {
                default:

                    break;
#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
                case RX_PROVIDER_PPM:
                    if (isPPMDataBeingReceived()) {
                        signalReceived = true;
                        resetPPMDataReceivedState();
                    }

                    break;
                case RX_PROVIDER_PARALLEL_PWM:
                    if (isPWMDataBeingReceived()) {
                        signalReceived = true;
                        useDataDrivenProcessing = false;
                    }

                    break;
#endif
                case RX_PROVIDER_SERIAL:
                case RX_PROVIDER_MSP:
                case RX_PROVIDER_SPI: {
                    const uint8_t frameStatus = rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);
                    // DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 1, (frameStatus & RX_FRAME_FAILSAFE));
                    signalReceived = (frameStatus & RX_FRAME_COMPLETE) &&
                                     !(frameStatus & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED));
                    setLinkQuality(signalReceived, currentDeltaTimeUs);
                    auxiliaryProcessingRequired |= (frameStatus & RX_FRAME_PROCESSING_REQUIRED);
                }

                break;
            }

            if (signalReceived) {
                // 仅当新数据包到达时为true
                needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
                rxSignalReceived = true;  // 立即处理数据包数据
                if (useDataDrivenProcessing) {
                    rxDataProcessingRequired = true;
                    // 当数据包到达时处理新的接收数据包
                }
            } else {
                // 等待下一个数据包
                if (CMP<timeDelta_t>(currentTimeUs, needRxSignalBefore) > 0) {
                    // 初始信号接收失败时间为100ms，然后每100ms检查一次
                    rxSignalReceived = false;
                    needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
                    // 每100ms检查并处理rcData值，以防失效保护改变了它们
                    rxDataProcessingRequired = true;
                }
            }

            // DEBUG_SET(DEBUG_FAILSAFE, 1, rxSignalReceived);
            // DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 0, rxSignalReceived);
        }

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
        static uint16_t calculateChannelMovingAverage(uint8_t chan, uint16_t sample) {
            static int16_t rcSamples[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT]
                                    [PPM_AND_PWM_SAMPLE_COUNT];
            static int16_t rcDataMean[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT];
            static bool rxSamplesCollected = false;

            const uint8_t currentSampleIndex = rcSampleIndex % PPM_AND_PWM_SAMPLE_COUNT;

            // 更新最近的采样并计算它们的平均值
            rcSamples[chan][currentSampleIndex] = sample;

            // 在采样足够之前避免返回不正确的平均值
            if (!rxSamplesCollected) {
                if (rcSampleIndex < PPM_AND_PWM_SAMPLE_COUNT) {
                    return sample;
                }
                rxSamplesCollected = true;
            }

            rcDataMean[chan] = 0;
            for (int sampleIndex = 0; sampleIndex < PPM_AND_PWM_SAMPLE_COUNT; sampleIndex++) {
                rcDataMean[chan] += rcSamples[chan][sampleIndex];
            }
            return rcDataMean[chan] / PPM_AND_PWM_SAMPLE_COUNT;
        }
#endif

        static uint16_t getRxfailValue(uint8_t channel) {
            const rxFailsafeChannelConfig_t* channelFailsafeConfig =
                rxFailsafeChannelConfigs(channel);
            const bool boxFailsafeSwitchIsOn = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
            UNUSED(boxFailsafeSwitchIsOn);

            switch (channelFailsafeConfig->mode) {
                case RX_FAILSAFE_MODE_AUTO:
                    // todo: 对于RoboMaster机器人，所有摇杆通道都使用中值作为失效保护值
                    // todo: 因为它们都用于控制方向
                    switch (channel) {
                        case ROLL:
                            if (rxfail_handlers.roll_handler) {
                                return rxfail_handlers.roll_handler();
                            }
                            break;
                        case PITCH:
                            if (rxfail_handlers.pitch_handler) {
                                return rxfail_handlers.pitch_handler();
                            }
                            break;
                        case YAW:
                            if (rxfail_handlers.yaw_handler) {
                                return rxfail_handlers.yaw_handler();
                            }
                            break;
                        case THROTTLE:
                            if (rxfail_handlers.throttle_handler) {
                                return rxfail_handlers.throttle_handler();
                            }
                            break;
                    }

                    FALLTHROUGH;
                default:
                case RX_FAILSAFE_MODE_INVALID:
                case RX_FAILSAFE_MODE_HOLD:
                    if (boxFailsafeSwitchIsOn) {
                        return rcRaw[channel];  // 当前值允许通过保持的通道
                                                // 使用开关触发的失效保护
                    } else {
                        return rcData[channel];  // 上一个良好值
                    }
                case RX_FAILSAFE_MODE_SET:
                    return RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfig->step);
            }
        }

        STATIC_UNIT_TESTED float applyRxChannelRangeConfiguraton(
            float sample, const rxChannelRangeConfig_t* range) {
            // 避免用PPM_RCVR_TIMEOUT值损坏通道
            if (sample == PPM_RCVR_TIMEOUT) {
                return PPM_RCVR_TIMEOUT;
            }

            sample =
                scaleRange<float>(sample, range->min, range->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
            // 在detectAndApplySignalLossBehaviour()中的有效性检查后，超出范围的通道值现在被约束
            return sample;
        }

        static void readRxChannelsApplyRanges(void) {
            for (int channel = 0; channel < rxChannelCount; channel++) {
                const uint8_t rawChannel =
                    channel < RX_MAPPABLE_CHANNEL_COUNT ? RPYTAConfig()->rcmap[channel] : channel;

                // 采样通道
                float sample;
#if defined(USE_RX_MSP_OVERRIDE)
                if (rxConfig()->msp_override_channels_mask) {
                    sample = rxMspOverrideReadRawRc(&rxRuntimeState, rxConfig(), rawChannel);
                } else
#endif
                {
                    sample = rxRuntimeState.rcReadRawFn(&rxRuntimeState, rawChannel);
                }

                // 应用接收机校准
                if (channel < NON_AUX_CHANNEL_COUNT) {
                    sample =
                        applyRxChannelRangeConfiguraton(sample, rxChannelRangeConfigs(channel));
                }

                rcRaw[channel] = sample;
            }
        }

        void detectAndApplySignalLossBehaviour(void) {
            const uint32_t currentTimeMs = bsp::GetHighresTickMilliSec();
            const bool boxFailsafeSwitchIsOn = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
            rxFlightChannelsValid = rxSignalReceived && !boxFailsafeSwitchIsOn;
            // 在100ms无数据包后rxFlightChannelsValid为false，或一旦使用
            // BOXFAILSAFE开关被操作，rxFlightChannelsValid在我们获得
            // 好数据包或BOXFAILSAFE开关恢复时立即为true，也可能在有好数据包时
            // 但其中一个飞行通道坏>300ms时为false（PPM类型接收机错误）

            for (int channel = 0; channel < rxChannelCount; channel++) {
                float sample = rcRaw[channel];  // sample为最新RC值，rcData为上一个
                                                // '接受的有效'值
                const bool thisChannelValid = rxFlightChannelsValid && isPulseValid(sample);
                // 如果整个数据包坏，或BOXFAILSAFE开关被操作，则认为所有
                // 通道都坏
                if (thisChannelValid) {
                    // 为每个好通道重置无效脉冲周期计时器
                    validRxSignalTimeout[channel] = currentTimeMs + MAX_INVALID_PULSE_TIME_MS;
                }

                if (failsafeIsActive()) {
                    // 我们处于失效保护第2阶段，无论是接收丢失还是BOXFAILSAFE触发
                    // 传递有效的传入飞行通道值到飞控，
                    // 以便GPS救援可以获得30%的要求来终止救援
                    if (channel < NON_AUX_CHANNEL_COUNT) {
                        if (!thisChannelValid) {
                            sample = rxConfig()->midrc;
                            // todo: 可以全部保持中值
                            // if (channel == THROTTLE) {
                            //     sample = failsafeConfig()->failsafe_throttle;
                            //     // 第2阶段失效保护油门值。在GPS救援飞行模式中，
                            //     // gpsRescueGetThrottle会在mixer.c中覆盖
                            // } else {
                            //     sample = rxConfig()->midrc;
                            // }
                        }
                    } else {
                        // 根据第1阶段失效保护保持/设置值设置辅助通道，允许所有
                        // 失效保护和GPS救援模式开关
                        sample = getRxfailValue(channel);
                    }
                } else {
                    // 我们正常，或处于失效保护第1阶段
                    if (boxFailsafeSwitchIsOn) {
                        // BOXFAILSAFE激活，但尚未进入第2阶段，使用第1阶段值
                        sample = getRxfailValue(channel);
                        // 一旦失效保护开关激活，立即将通道设置为第1阶段值
                    } else if (!thisChannelValid) {
                        // 一切正常但此通道无效
                        if (CMP<int32_t>(currentTimeMs, validRxSignalTimeout[channel]) < 0) {
                            // 第1阶段失效保护的前300ms
                            sample = rcData[channel];
                            // 在MAX_INVALID_PULSE_TIME_MS（300ms）期间保持
                            // 上一个有效值在坏通道上
                        } else {
                            // 300ms后的第1阶段失效保护期间
                            if (channel < NON_AUX_CHANNEL_COUNT) {
                                rxFlightChannelsValid = false;
                                // 在任何一个坏飞行通道持续300ms后声明信号丢失
                            }
                            sample = getRxfailValue(channel);
                            // 将超过300ms无效的通道设置为第1阶段值
                        }
                    }
                    // 一切正常，即rcData[channel]将通过'sample'设置为rcRaw(channel)
                }

                sample = constrain<float>(sample, PWM_PULSE_MIN, PWM_PULSE_MAX);

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
                if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM ||
                    rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
                    // 使用移动平均平滑PWM和PPM输出
                    rcData[channel] = calculateChannelMovingAverage(channel, sample);
                } else
#endif

                {
                    // 将rcData设置为验证过的传入值，或失效保护修改后的值
                    rcData[channel] = sample;
                }
            }

            if (rxFlightChannelsValid) {
                failsafeOnValidDataReceived();
                // --> 在丢失所有数据包或BOXFAILSAFE开关被操作后100ms开始计时器退出第2阶段失效保护
            } else {
                failsafeOnValidDataFailed();
                // -> 在我们获得好数据包或BOXFAILSAFE开关恢复的瞬间开始计时器进入第2阶段失效保护
            }

            // DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 3, rcData[THROTTLE]);
        }

        bool calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs) {
            if (auxiliaryProcessingRequired) {
                auxiliaryProcessingRequired = !rxRuntimeState.rcProcessFrameFn(&rxRuntimeState);
            }

            if (!rxDataProcessingRequired) {
                return false;
            }

            rxDataProcessingRequired = false;

            // 仅在没有更多采样要跳过且暂停期结束后继续
            if (skipRxSamples || currentTimeUs <= suspendRxSignalUntil) {
                if (currentTimeUs > suspendRxSignalUntil) {
                    skipRxSamples--;
                }

                return true;
            }

            readRxChannelsApplyRanges();          // 返回rcRaw
            detectAndApplySignalLossBehaviour();  // 返回rcData

            rcSampleIndex++;

            return true;
        }

        void parseRcChannels(const char* input, RPYTAConfig_t* RPYTAConfig) {
            for (const char* c = input; *c; c++) {
                const char* s = strchr(rcChannelLetters, *c);
                if (s && (s < rcChannelLetters + RX_MAPPABLE_CHANNEL_COUNT)) {
                    RPYTAConfig->rcmap[s - rcChannelLetters] = c - input;
                }
            }
        }

        ///
        ///      RSSI 模块
        ///

        void setRssiDirect(uint16_t newRssi, rssiSource_e source) {
            if (source != rssiSource) {
                return;
            }

            rssi = newRssi;
            rssiRaw = newRssi;
        }

        void setRssi(uint16_t rssiValue, rssiSource_e source) {
            if (source != rssiSource) {
                return;
            }

            // 滤波RSSI值
            if (source == RSSI_SOURCE_FRAME_ERRORS) {
                rssiRaw = PT1FilterUpdate(&frameErrFilter, rssiValue);
            } else {
                rssiRaw = rssiValue;
            }
        }

        void setRssiMsp(uint8_t newMspRssi) {
            if (rssiSource == RSSI_SOURCE_NONE) {
                rssiSource = RSSI_SOURCE_MSP;
            }

            if (rssiSource == RSSI_SOURCE_MSP) {
                rssi = ((uint16_t)newMspRssi) << 2;
                lastMspRssiUpdateUs = bsp::GetHighresTickMicroSec();
            }
        }

        static void updateRSSIPWM(void) {
            // 将辅助通道的值作为RSSI读取
            int16_t pwmRssi = rcData[rxConfig()->rssi_channel - 1];

            // rawPwmRssi的范围是[1000;2000]。rssi应在[0;1023]范围内；
            setRssiDirect(scaleRange<int>(constrain<int>(pwmRssi, PWM_RANGE_MIN, PWM_RANGE_MAX),
                                          PWM_RANGE_MIN, PWM_RANGE_MAX, 0, RSSI_MAX_VALUE),
                          RSSI_SOURCE_RX_CHANNEL);
        }

        static void updateRSSIADC(timeUs_t currentTimeUs) {
#ifndef USE_ADC
            UNUSED(currentTimeUs);
#else
            static uint32_t rssiUpdateAt = 0;

            if ((int32_t)(currentTimeUs - rssiUpdateAt) < 0) {
                return;
            }
            rssiUpdateAt = currentTimeUs + DELAY_20_MS;

            const uint16_t adcRssiSample = adcGetChannel(ADC_RSSI);
            uint16_t rssiValue = adcRssiSample / RSSI_ADC_DIVISOR;

            setRssi(rssiValue, RSSI_SOURCE_ADC);
#endif
        }

        void updateRSSI(timeUs_t currentTimeUs) {
            switch (rssiSource) {
                case RSSI_SOURCE_RX_CHANNEL:
                    updateRSSIPWM();
                    break;
                case RSSI_SOURCE_ADC:
                    updateRSSIADC(currentTimeUs);
                    break;
                case RSSI_SOURCE_MSP:
                    if (CMP<timeDelta_t>(bsp::GetHighresTickMicroSec(), lastMspRssiUpdateUs) >
                        DELAY_1500_MS) {  // 1.5秒
                        rssi = 0;
                    }
                    break;
                default:
                    break;
            }

            if (CMP<timeDelta_t>(currentTimeUs, lastRssiSmoothingUs) > 250000) {  // 0.25秒
                lastRssiSmoothingUs = currentTimeUs;
            } else {
                if (lastRssiSmoothingUs != currentTimeUs) {  // 避免除零
                    float k = (256.0f - rxConfig()->rssi_smoothing) / 256.0f;
                    float factor =
                        ((currentTimeUs - lastRssiSmoothingUs) / 1000000.0f) / (1.0f / 4.0f);
                    float k2 = (k * factor) / ((k * factor) - k + 1);

                    if (rssi != rssiRaw) {
                        PT1FilterUpdateCutoff(&rssiFilter, k2);
                        rssi = PT1FilterUpdate(&rssiFilter, rssiRaw);
                    }

#ifdef USE_RX_RSSI_DBM
                    if (rssiDbm != rssiDbmRaw) {
                        PT1FilterUpdateCutoff(&rssiDbmFilter, k2);
                        rssiDbm = PT1FilterUpdate(&rssiDbmFilter, rssiDbmRaw);
                    }
#endif  // USE_RX_RSSI_DBM

#ifdef USE_RX_RSNR
                    if (rsnr != rsnrRaw) {
                        PT1FilterUpdateCutoff(&rsnrFilter, k2);
                        rsnr = PT1FilterUpdate(&rsnrFilter, rsnrRaw);
                    }
#endif  // USE_RX_RSNR

                    lastRssiSmoothingUs = currentTimeUs;
                }
            }
        }

        uint16_t getRssi(void) {
            uint16_t rssiValue = rssi;

            // RSSI反转选项
            if (rxConfig()->rssi_invert) {
                rssiValue = RSSI_MAX_VALUE - rssiValue;
            }

            return rxConfig()->rssi_scale / 100.0f * rssiValue +
                   rxConfig()->rssi_offset * RSSI_OFFSET_SCALING;
        }

        uint8_t getRssiPercent(void) {
            return scaleRange<int>(getRssi(), 0, RSSI_MAX_VALUE, 0, 100);
        }

#ifdef USE_RX_RSSI_DBM
        int16_t getRssiDbm(void) {
            return rssiDbm;
        }

        void setRssiDbm(int16_t rssiDbmValue, rssiSource_e source) {
            if (source != rssiSource) {
                return;
            }

            rssiDbmRaw = rssiDbmValue;
        }

        void setRssiDbmDirect(int16_t newRssiDbm, rssiSource_e source) {
            if (source != rssiSource) {
                return;
            }

            rssiDbm = newRssiDbm;
            rssiDbmRaw = newRssiDbm;
        }
#endif  // USE_RX_RSSI_DBM

#ifdef USE_RX_RSNR
        int16_t getRsnr(void) {
            return rsnr;
        }

        void setRsnr(int16_t rsnrValue) {
            rsnrRaw = rsnrValue;
        }

        void setRsnrDirect(int16_t newRsnr) {
            rsnr = newRsnr;
            rsnrRaw = newRsnr;
        }
#endif  // USE_RX_RSNR

#ifdef USE_RX_LINK_QUALITY_INFO
        uint16_t rxGetLinkQuality(void) {
            return linkQuality;
        }

        uint8_t rxGetRfMode(void) {
            return rfMode;
        }

        uint16_t rxGetLinkQualityPercent(void) {
            return (linkQualitySource == LQ_SOURCE_NONE)
                       ? scaleRange<int>(linkQuality, 0, LINK_QUALITY_MAX_VALUE, 0, 100)
                       : linkQuality;
        }
#endif

#ifdef USE_RX_LINK_UPLINK_POWER
        uint16_t rxGetUplinkTxPwrMw(void) {
            return uplinkTxPwrMw;
        }
#endif

        bool isRssiConfigured(void) {
            return rssiSource != RSSI_SOURCE_NONE;
        }

        timeDelta_t rxGetFrameDelta(timeDelta_t* frameAgeUs) {
            static timeUs_t previousFrameTimeUs = 0;
            static timeDelta_t frameTimeDeltaUs = 0;

            if (rxRuntimeState.rcFrameTimeUsFn) {
                const timeUs_t frameTimeUs = rxRuntimeState.rcFrameTimeUsFn();

                *frameAgeUs = CMP<timeDelta_t>(bsp::GetHighresTickMicroSec(), frameTimeUs);

                const timeDelta_t deltaUs = CMP<timeDelta_t>(frameTimeUs, previousFrameTimeUs);
                if (deltaUs) {
                    frameTimeDeltaUs = deltaUs;
                    previousFrameTimeUs = frameTimeUs;
                }
            }

            return frameTimeDeltaUs;
        }

        timeUs_t rxFrameTimeUs(void) {
            return rxRuntimeState.lastRcFrameTimeUs;
        }

    }  // namespace RX
}  // namespace RMFC