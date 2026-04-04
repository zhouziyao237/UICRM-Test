/*###########################################################
 # Copyright (c) 2023-2026. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 ###########################################################*/

#include "sbus.h"

#include <cstdlib>

#include "bsp_os.h"
#include "utils.h"

#define SBUS_RC_ROCKER_MID 1024
#define SBUS_RC_ROCKER_ZERO_DRIFT 10

namespace remote {

    namespace {
        constexpr uint8_t kFlagIndex = 23;
        constexpr uint8_t kEndIndex = 24;
        constexpr uint16_t kChannelMask = 0x07FF;
        constexpr uint8_t kChannelCount = 16;
    }  // namespace

    SBUS::SBUS(UART_HandleTypeDef* huart) : SBUS(huart, LinkGuardConfig{}) {
    }

    SBUS::SBUS(UART_HandleTypeDef* huart, const LinkGuardConfig& config)
        : bsp::UART(huart),
          driver::ConnectionDriver(ONLINE_TIMEOUT_MS),
          link_guard_config_(config) {
        // One extra byte lets the parser recover from a single leading noise
        // byte without treating UART activity as a valid heartbeat.
        SetupRx(FRAME_SIZE + 1);
        ClearChannels();
        ResetLinkGuardState();
    }

    bool SBUS::IsOnline() const {
        return !failsafe_ && !centered_failsafe_ && !sentinel_failsafe_ &&
               driver::ConnectionDriver::IsOnline();
    }

    void SBUS::ConfigureLinkGuard(const LinkGuardConfig& config) {
        link_guard_config_ = config;
        if (link_guard_config_.sentinel_channel > kChannelCount) {
            link_guard_config_.sentinel_channel = 0;
        }
        if (link_guard_config_.sentinel_min_abs < 0) {
            link_guard_config_.sentinel_min_abs =
                static_cast<int16_t>(-link_guard_config_.sentinel_min_abs);
        }
        ResetLinkGuardState();
        last_uptime_ = 0;
        ClearChannels();
    }

    void SBUS::ConfigureSentinelChannel(uint8_t channel,
                                        int16_t minimum_absolute_value,
                                        uint32_t confirm_ms) {
        link_guard_config_.sentinel_channel =
            channel <= kChannelCount ? channel : 0;
        link_guard_config_.sentinel_min_abs =
            minimum_absolute_value < 0
                ? static_cast<int16_t>(-minimum_absolute_value)
                : minimum_absolute_value;
        link_guard_config_.sentinel_confirm_ms = confirm_ms;
        ResetLinkGuardState();
        last_uptime_ = 0;
        ClearChannels();
    }

    void SBUS::DisableSentinelChannel() {
        link_guard_config_.sentinel_channel = 0;
        ResetLinkGuardState();
    }

    bool SBUS::IsAcceptedEndByte(uint8_t value) {
        // 0x00 is the normal SBUS end byte. 0x04/0x14/0x24/0x34 are
        // commonly used SBUS2 slot markers and still delimit a valid frame.
        return value == 0x00 || value == 0x04 || value == 0x14 ||
               value == 0x24 || value == 0x34;
    }

    uint16_t SBUS::DecodeChannel(const uint8_t* frame, uint8_t channel) {
        // Sixteen 11-bit channels start at frame byte 1. Decode by bit offset
        // instead of relying on implementation-defined C/C++ bitfield layout.
        const uint16_t bit_offset = static_cast<uint16_t>(channel) * 11U;
        const uint8_t byte_index = static_cast<uint8_t>(1U + bit_offset / 8U);
        const uint8_t shift = static_cast<uint8_t>(bit_offset % 8U);

        uint32_t packed = frame[byte_index];
        if (byte_index + 1U < kFlagIndex) {
            packed |= static_cast<uint32_t>(frame[byte_index + 1U]) << 8U;
        }
        if (byte_index + 2U < kFlagIndex) {
            packed |= static_cast<uint32_t>(frame[byte_index + 2U]) << 16U;
        }

        return static_cast<uint16_t>((packed >> shift) & kChannelMask);
    }

    int16_t SBUS::NormalizeChannel(uint16_t raw) {
        int16_t value = clip<int16_t>(
            static_cast<int16_t>(raw) - SBUS_RC_ROCKER_MID, -660, 660);
        if (std::abs(value) <= SBUS_RC_ROCKER_ZERO_DRIFT) {
            value = 0;
        }
        return value;
    }

    const uint8_t* SBUS::FindFrame(const uint8_t* data, uint32_t length) const {
        if (data == nullptr || length < FRAME_SIZE) {
            return nullptr;
        }

        for (uint32_t offset = 0; offset + FRAME_SIZE <= length; ++offset) {
            const uint8_t* candidate = data + offset;
            if (candidate[0] == START_BYTE &&
                IsAcceptedEndByte(candidate[kEndIndex])) {
                return candidate;
            }
        }
        return nullptr;
    }

    void SBUS::DecodeRawChannels(const uint8_t* frame,
                                 uint16_t* raw_channels) const {
        if (frame == nullptr || raw_channels == nullptr) {
            return;
        }
        for (uint8_t i = 0; i < kChannelCount; ++i) {
            raw_channels[i] = DecodeChannel(frame, i);
        }
    }

    void SBUS::ClearChannels() {
        ch1 = 0;
        ch2 = 0;
        ch3 = 0;
        ch4 = 0;
        ch5 = 0;
        ch6 = 0;
        ch7 = 0;
        ch8 = 0;
        ch9 = 0;
        ch10 = 0;
        ch11 = 0;
        ch12 = 0;
        ch13 = 0;
        ch14 = 0;
        ch15 = 0;
        ch16 = 0;
    }

    void SBUS::ApplyChannels(const uint16_t* raw_channels) {
        if (raw_channels == nullptr) {
            return;
        }
        ch1 = NormalizeChannel(raw_channels[0]);
        ch2 = NormalizeChannel(raw_channels[1]);
        ch3 = NormalizeChannel(raw_channels[2]);
        ch4 = NormalizeChannel(raw_channels[3]);
        ch5 = NormalizeChannel(raw_channels[4]);
        ch6 = NormalizeChannel(raw_channels[5]);
        ch7 = NormalizeChannel(raw_channels[6]);
        ch8 = NormalizeChannel(raw_channels[7]);
        ch9 = NormalizeChannel(raw_channels[8]);
        ch10 = NormalizeChannel(raw_channels[9]);
        ch11 = NormalizeChannel(raw_channels[10]);
        ch12 = NormalizeChannel(raw_channels[11]);
        ch13 = NormalizeChannel(raw_channels[12]);
        ch14 = NormalizeChannel(raw_channels[13]);
        ch15 = NormalizeChannel(raw_channels[14]);
        ch16 = NormalizeChannel(raw_channels[15]);
    }

    void SBUS::ResetLinkGuardState() {
        centered_failsafe_ = false;
        sentinel_failsafe_ = false;
        centered_since_ms_ = 0;
        sentinel_bad_since_ms_ = 0;
    }

    bool SBUS::IsAllChannelsCentered(const uint16_t* raw_channels) const {
        if (!link_guard_config_.centered_failsafe_enabled ||
            raw_channels == nullptr) {
            return false;
        }

        const int32_t midpoint = link_guard_config_.centered_raw_value;
        const int32_t tolerance = link_guard_config_.centered_tolerance;
        for (uint8_t i = 0; i < kChannelCount; ++i) {
            const int32_t error =
                static_cast<int32_t>(raw_channels[i]) - midpoint;
            if (error > tolerance || error < -tolerance) {
                return false;
            }
        }
        return true;
    }

    bool SBUS::PassesSentinel(const uint16_t* raw_channels) const {
        const uint8_t channel = link_guard_config_.sentinel_channel;
        if (channel == 0 || channel > kChannelCount) {
            return true;
        }
        if (raw_channels == nullptr) {
            return false;
        }

        const int16_t value = NormalizeChannel(raw_channels[channel - 1U]);
        return std::abs(value) >= link_guard_config_.sentinel_min_abs;
    }

    bool SBUS::UpdateLinkGuard(const uint16_t* raw_channels, uint64_t now_ms) {
        // Receiver signature fallback: every channel exactly returns to the
        // midpoint. Confirmation avoids killing on a single neutral frame.
        if (IsAllChannelsCentered(raw_channels)) {
            if (centered_since_ms_ == 0) {
                centered_since_ms_ = now_ms;
            }
            if (now_ms - centered_since_ms_ >=
                link_guard_config_.centered_confirm_ms) {
                centered_failsafe_ = true;
            }
        } else {
            centered_since_ms_ = 0;
            centered_failsafe_ = false;
        }

        // Dedicated sentinel is the recommended unambiguous link proof.
        if (!PassesSentinel(raw_channels)) {
            if (sentinel_bad_since_ms_ == 0) {
                sentinel_bad_since_ms_ = now_ms;
            }
            if (now_ms - sentinel_bad_since_ms_ >=
                link_guard_config_.sentinel_confirm_ms) {
                sentinel_failsafe_ = true;
            }
        } else {
            sentinel_bad_since_ms_ = 0;
            sentinel_failsafe_ = false;
        }

        return !centered_failsafe_ && !sentinel_failsafe_;
    }

    void SBUS::RxCompleteCallback() {
        uint8_t* data = nullptr;
        const int32_t length = Read<true>(&data);
        if (length <= 0) {
            return;
        }

        const uint8_t* frame = FindFrame(data, static_cast<uint32_t>(length));
        if (frame == nullptr) {
            ++invalid_frame_count_;
            // Noise and partial frames must age out naturally.
            return;
        }

        flag = frame[kFlagIndex];
        frame_lost_ = (flag & FLAG_FRAME_LOST) != 0U;
        failsafe_ = (flag & FLAG_FAILSAFE) != 0U;

        if (failsafe_) {
            ClearChannels();
            ResetLinkGuardState();
            last_uptime_ = 0;
            return;
        }

        if (frame_lost_) {
            // Keep the previous good heartbeat for one-off lost frames;
            // persistent lost frames time out through ONLINE_TIMEOUT_MS.
            return;
        }

        uint16_t raw_channels[kChannelCount] = {};
        DecodeRawChannels(frame, raw_channels);

        const uint64_t now_ms = bsp::GetHighresTickMilliSec();
        if (!UpdateLinkGuard(raw_channels, now_ms)) {
            ClearChannels();
            last_uptime_ = 0;
            return;
        }

        ApplyChannels(raw_channels);
        Heartbeat();
        timestamp = static_cast<uint32_t>(GetLastUptime());
        ++valid_frame_count_;
    }

}  // namespace remote
