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
// Created by Administrator on 2025/8/23.
//

#include "platform.h"
#include "target.h"

#if defined(USE_RX_PWM) || defined(USE_RX_PPM) || defined(USE_SERIALRX) || defined(USE_RX_MSP) || defined(USE_RX_SPI)

#include "rx.h"
#include "Module/RX/rx.h"
#include "Module/RX/rx_config.h"
#include "Device/bsp_PG/bsp_pg.h"
#include "Device/bsp_PG/bsp_pg_ids.h"

namespace RMFC {
    namespace RX {
        PG_REGISTER_WITH_RESET_FN(rxConfig_t, rxConfig, PG_RX_CONFIG, 4);
        void pgResetFn_rxConfig(rxConfig_t *rxConfig, int size)
        {
            UNUSED(size);
            RESET_CONFIG_2(rxConfig_t, rxConfig,
                // .rcmap = {0}, // todo: 后面单独写一个结构体去这里会影响到下面的字节对其和数据
                .serialrx_provider = SERIALRX_PROVIDER,
                .serialrx_inverted = 0,
                .halfDuplex = 0,
                // .spektrum_bind_pin_override_ioTag = 0,
                // .spektrum_bind_plug_ioTag = 0,
                // .spektrum_sat_bind = 0,
                // .spektrum_sat_bind_autoreset = 1,
                .rssi_channel = 0,
                .rssi_scale = RSSI_SCALE_DEFAULT,
                .rssi_invert = 0,
                .midrc = RX_MID_USEC,
                .mincheck = 1050,
                .maxcheck = 1900,
                .fpvCamAngleDegrees = 0,
                .airModeActivateThreshold = 25,
                .rx_min_usec = RX_MIN_USEC,
                .rx_max_usec = RX_MAX_USEC,
                .max_aux_channel = DEFAULT_AUX_CHANNEL_COUNT,
                .rssi_src_frame_errors = false,
                .rssi_offset = 0,
                .rc_smoothing_mode = 1,
                .rc_smoothing_setpoint_cutoff = 0,
                .rc_smoothing_feedforward_cutoff = 0,
                .rc_smoothing_throttle_cutoff = 0,
                .rc_smoothing_debug_axis = RMFC::ROLL,
                .rc_smoothing_auto_factor_rpy = 30,
                .rc_smoothing_auto_factor_throttle = 30,
                .rssi_src_frame_lpf_period = 30,
                .rssi_smoothing = 125,
                .srxl2_unit_id = 1,
                .srxl2_baud_fast = true,
                .sbus_baud_fast = false,
                .dbus_baud_fast = false,
                .msp_override_channels_mask = 0,
                .crsf_use_negotiated_baud = false,
                );
        }

        // --------------------------------------- 分割线 ---------------------------------------

        PG_REGISTER_WITH_RESET_FN(RPYTAConfig_t, RPYTAConfig, PG_RPYTA_CONFIG, 4);
        void pgResetFn_RPYTAConfig(RPYTAConfig_t* RPYTAConfig, int size) {
            UNUSED(size);
            RESET_CONFIG_2(RPYTAConfig_t, RPYTAConfig,
                .rcmap = {0},  // todo: 后面单独写一个结构体去这里会影响到下面的字节对其和数据
            );

            for (int i = 0; i < RX_MAPPABLE_CHANNEL_COUNT; i++) {
                RPYTAConfig->rcmap[i] = 0;
            }

#ifdef RX_CHANNELS_TAER
            parseRcChannels("AETR1234", RPYTAConfig);
#else
            parseRcChannels("AETR1234", RPYTAConfig);
#endif
    }

    }
}
#endif