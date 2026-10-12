#include "power_task.h"

#include <cmath>

#include "atlas_config.h"
#include "chassis_task.h"
#include "referee_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "utils.h"

namespace {

PowerForwardTelemetry_t g_telemetry{};

constexpr uint32_t kPowerForwardPollMs = 1U;
// 0x72 is a state/heartbeat frame.  A 20 ms heartbeat keeps the lower board
// able to detect a dead/stalled upper-board forwarder without flooding CAN.
constexpr uint32_t kPowerLimitHeartbeatMs = 20U;

uint32_t g_last_limit_forward_ms = 0U;
uint32_t g_last_forwarded_power_sequence = 0U;
bool g_forward_initialized = false;
bool g_previous_referee_online = false;
uint8_t g_previous_limit_w = 0U;
uint8_t g_previous_buffer_joule = 0U;
bool g_previous_failsafe = true;

struct ForwardState {
    uint8_t max_watt = kInvalidRefereePowerLimitW;
    float buffer_joule = kInvalidRefereeBufferJoule;
    float measured_power = -1.0f;
    bool referee_online = false;
    bool referee_limit_valid = false;
    bool local_test = false;
    bool failsafe = true;
    uint32_t power_sequence = 0U;
};

ForwardState BuildForwardState(const RefereeSnapshot_t& referee) {
    ForwardState state;
    state.power_sequence = referee.power_heat_sequence;

#ifdef HAS_REFEREE
    if (referee.online) {
        state.referee_online = true;
        state.referee_limit_valid =
            referee.chassis_power_limit > 0U &&
            static_cast<float>(referee.chassis_power_limit) <=
                kMaxValidRefereePowerW &&
            referee.chassis_power_buffer <= 60U;

        if (state.referee_limit_valid) {
            state.max_watt =
                static_cast<uint8_t>(referee.chassis_power_limit);
            state.buffer_joule =
                static_cast<float>(referee.chassis_power_buffer);
            state.failsafe = false;
            if (std::isfinite(referee.chassis_power) &&
                referee.chassis_power >= 0.0f) {
                state.measured_power = referee.chassis_power;
            }
        }
        // Online-but-invalid data intentionally stays at the conservative
        // 45 W / 0 J fail-safe.  Never replace it with a bench fallback.
        return state;
    }
#endif

#if ATLAS_ENABLE_NO_REFEREE_POWER_TEST
    state.max_watt = kLocalChassisPowerLimitW;
    state.buffer_joule = kLocalChassisBufferJoule;
    state.local_test = true;
    state.failsafe = false;
#endif
    return state;
}

void PublishTelemetry(const ForwardState& state, uint32_t now_ms) {
    PowerForwardTelemetry_t snapshot;
    snapshot.initialized = true;
    snapshot.referee_online = state.referee_online;
    snapshot.power_from_referee = state.referee_limit_valid;
    snapshot.power_local_test = state.local_test;
    snapshot.power_failsafe = state.failsafe;
    snapshot.power_limit_w = static_cast<float>(state.max_watt);
    snapshot.power_buffer_joule = state.buffer_joule;
    snapshot.measured_power_w = state.measured_power;
    snapshot.referee_power_sequence = state.power_sequence;
    snapshot.forwarded_power_sequence = g_last_forwarded_power_sequence;
    snapshot.last_limit_forward_ms = g_last_limit_forward_ms;
    snapshot.last_power_forward_ms = g_telemetry.last_power_forward_ms;

    // One small coherent application snapshot; do not hold this gate while
    // sending CAN or reading peripherals.
    taskENTER_CRITICAL();
    g_telemetry = snapshot;
    taskEXIT_CRITICAL();
    UNUSED(now_ms);
}

}  // namespace

osThreadId_t PowerTaskHandle = nullptr;

void InitPower() {
    // Upper board only forwards referee power state.
    // Chassis-bus voltage/current are sampled exclusively on the lower board.
}

void PowerTask(void* arg) {
    UNUSED(arg);

    while (true) {
        const uint32_t now_ms = HAL_GetTick();
        const RefereeSnapshot_t referee = RefereeGetSnapshot();
        const ForwardState state = BuildForwardState(referee);

        const uint8_t rounded_buffer = static_cast<uint8_t>(
            state.buffer_joule < 0.0f
                ? 0.0f
                : (state.buffer_joule > 60.0f ? 60.0f
                                               : state.buffer_joule + 0.5f));

        const bool state_changed =
            !g_forward_initialized ||
            state.referee_online != g_previous_referee_online ||
            state.max_watt != g_previous_limit_w ||
            rounded_buffer != g_previous_buffer_joule ||
            state.failsafe != g_previous_failsafe;
        const bool heartbeat_due =
            !g_forward_initialized ||
            static_cast<uint32_t>(now_ms - g_last_limit_forward_ms) >=
                kPowerLimitHeartbeatMs;
        const bool new_power_sample =
            state.referee_online &&
            state.power_sequence != g_last_forwarded_power_sequence;

        // Send 0x72 first whenever a new Pr/Z sample arrives so the lower board
        // has the matching limit/source/buffer state before 0x73 is consumed.
        if (state_changed || heartbeat_due || new_power_sample) {
            ChassisForwardPowerLimit(true, state.max_watt,
                                     state.buffer_joule,
                                     state.referee_online);
            g_last_limit_forward_ms = now_ms;
            g_previous_referee_online = state.referee_online;
            g_previous_limit_w = state.max_watt;
            g_previous_buffer_joule = rounded_buffer;
            g_previous_failsafe = state.failsafe;
            g_forward_initialized = true;
        }

        // 0x73 is event-driven from the actual 0x0202 source sequence.  Equal
        // consecutive numeric Pr values are still forwarded exactly once each.
        if (new_power_sample) {
            ChassisForwardCurrentPower(state.measured_power,
                                       state.buffer_joule);
            g_last_forwarded_power_sequence = state.power_sequence;
            taskENTER_CRITICAL();
            g_telemetry.last_power_forward_ms = now_ms;
            taskEXIT_CRITICAL();
        }

        PublishTelemetry(state, now_ms);
        osDelay(kPowerForwardPollMs);
    }
}

PowerForwardTelemetry_t PowerForwardGetTelemetry() {
    taskENTER_CRITICAL();
    const PowerForwardTelemetry_t snapshot = g_telemetry;
    taskEXIT_CRITICAL();
    return snapshot;
}
