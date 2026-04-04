/*###########################################################
 # Copyright (c) 2023-2026. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even the       #
 # implied warranty of MERCHANTABILITY or FITNESS FOR A     #
 # PARTICULAR PURPOSE.  See the GNU General Public License  #
 # for more details.                                        #
 ###########################################################*/

#pragma once

#include <stdint.h>

#include "bsp_uart.h"
#include "connection_driver.h"

namespace remote {

    /**
     * @brief SBUS receiver with protocol-aware and receiver-failsafe-aware
     *        online detection.
     *
     * A receiver can remain powered and continue transmitting valid-looking
     * SBUS frames after the transmitter link is lost. Some receivers set the
     * SBUS failsafe bit; others output all channels at the raw midpoint. This
     * driver supports both behaviours and also provides an optional spare-
     * channel sentinel, which is the only unambiguous software-only solution
     * when the receiver emits ordinary midpoint frames without a failsafe bit.
     */
    class SBUS : public bsp::UART, public driver::ConnectionDriver {
      public:
        static constexpr uint8_t FRAME_SIZE = 25;
        static constexpr uint8_t START_BYTE = 0x0F;
        static constexpr uint8_t FLAG_CHANNEL_17 = 0x01;
        static constexpr uint8_t FLAG_CHANNEL_18 = 0x02;
        static constexpr uint8_t FLAG_FRAME_LOST = 0x04;
        static constexpr uint8_t FLAG_FAILSAFE = 0x08;
        static constexpr uint32_t ONLINE_TIMEOUT_MS = 100;

        /**
         * @brief Link guard configuration.
         *
         * centered_failsafe_enabled handles receivers which output every
         * channel at raw midpoint after link loss. It is enabled by default for
         * this project. A normal transmitter frame with every channel exactly
         * centred for centered_confirm_ms is therefore treated as unsafe. For
         * zero ambiguity, configure a spare channel (normally CH16) as a fixed
         * non-centred value and enable sentinel_channel.
         */
        struct LinkGuardConfig {
            bool centered_failsafe_enabled = true;
            uint16_t centered_raw_value = 1024;
            uint16_t centered_tolerance = 2;
            uint32_t centered_confirm_ms = 500;

            // 0 disables the sentinel. 1..16 selects a channel which must stay
            // away from centre while the transmitter link is alive.
            uint8_t sentinel_channel = 0;
            int16_t sentinel_min_abs = 200;
            uint32_t sentinel_confirm_ms = 100;
        };

        explicit SBUS(UART_HandleTypeDef* huart);
        SBUS(UART_HandleTypeDef* huart, const LinkGuardConfig& config);

        void RxCompleteCallback() override final;

        bool IsOnline() const;

        /**
         * @brief Update link-guard behaviour at runtime.
         *
         * Calling this resets the guard state. The next accepted frame starts
         * a fresh confirmation period.
         */
        void ConfigureLinkGuard(const LinkGuardConfig& config);

        /**
         * @brief Enable a dedicated link sentinel channel.
         *
         * Configure the transmitter so this spare channel is permanently high
         * or low while connected. When the receiver loses link and returns all
         * channels to centre, the sentinel becomes centred and the driver goes
         * offline after confirm_ms.
         */
        void ConfigureSentinelChannel(uint8_t channel,
                                      int16_t minimum_absolute_value = 200,
                                      uint32_t confirm_ms = 100);

        void DisableSentinelChannel();

        bool IsFailsafe() const {
            return failsafe_;
        }

        bool IsFrameLost() const {
            return frame_lost_;
        }

        bool IsCenteredFailsafe() const {
            return centered_failsafe_;
        }

        bool IsSentinelFailsafe() const {
            return sentinel_failsafe_;
        }

        uint32_t GetValidFrameCount() const {
            return valid_frame_count_;
        }

        uint32_t GetInvalidFrameCount() const {
            return invalid_frame_count_;
        }

        // Normalized analog channels. Existing users expect approximately
        // -660..660 around a raw SBUS midpoint of 1024.
        volatile int16_t ch1 = 0;
        volatile int16_t ch2 = 0;
        volatile int16_t ch3 = 0;
        volatile int16_t ch4 = 0;
        volatile int16_t ch5 = 0;
        volatile int16_t ch6 = 0;
        volatile int16_t ch7 = 0;
        volatile int16_t ch8 = 0;
        volatile int16_t ch9 = 0;
        volatile int16_t ch10 = 0;
        volatile int16_t ch11 = 0;
        volatile int16_t ch12 = 0;
        volatile int16_t ch13 = 0;
        volatile int16_t ch14 = 0;
        volatile int16_t ch15 = 0;
        volatile int16_t ch16 = 0;
        volatile uint8_t flag = 0;

        // Timestamp of the latest accepted online frame.
        uint32_t timestamp = 0;

        static const int16_t ROCKER_MIN = -1023;
        static const int16_t ROCKER_MAX = 1023;

      private:
        static bool IsAcceptedEndByte(uint8_t value);
        static uint16_t DecodeChannel(const uint8_t* frame, uint8_t channel);
        static int16_t NormalizeChannel(uint16_t raw);

        const uint8_t* FindFrame(const uint8_t* data, uint32_t length) const;
        void DecodeRawChannels(const uint8_t* frame, uint16_t* raw_channels) const;
        void ApplyChannels(const uint16_t* raw_channels);
        void ClearChannels();
        void ResetLinkGuardState();
        bool IsAllChannelsCentered(const uint16_t* raw_channels) const;
        bool PassesSentinel(const uint16_t* raw_channels) const;
        bool UpdateLinkGuard(const uint16_t* raw_channels, uint64_t now_ms);

        LinkGuardConfig link_guard_config_{};

        volatile bool failsafe_ = true;
        volatile bool frame_lost_ = true;
        volatile bool centered_failsafe_ = false;
        volatile bool sentinel_failsafe_ = false;
        volatile uint32_t valid_frame_count_ = 0;
        volatile uint32_t invalid_frame_count_ = 0;

        uint64_t centered_since_ms_ = 0;
        uint64_t sentinel_bad_since_ms_ = 0;
    };

}  // namespace remote
