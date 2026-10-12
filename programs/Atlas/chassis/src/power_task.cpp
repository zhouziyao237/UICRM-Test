#include "power_task.h"

#include <cmath>

#include "adc.h"
#include "bsp_batteryvol.h"
#include "chassis_task.h"
#include "power_control_profile.h"
#include "utils.h"

namespace {
bsp::BatteryVol* g_power_battery = nullptr;
PowerCurrentReader_t g_current_reader = nullptr;

constexpr uint32_t kPowerControlPeriodMs = 1U;

uint32_t g_last_bridge_sequence = 0U;
uint32_t g_last_bridge_update_ms = 0U;
bool g_bridge_seen = false;
bool g_bridge_stale_active = false;
}  // namespace

osThreadId_t PowerTaskHandle = nullptr;

void PowerControlRegisterCurrentReader(PowerCurrentReader_t reader) {
    g_current_reader = reader;
}

void InitPower() {
    // Atlas lower board is the sole owner of chassis-bus electrical sensing.
    // Voltage is available now; the current-reader callback is plugged in later.
    g_power_battery =
        new bsp::BatteryVol(&hadc3, ADC_CHANNEL_8, 1, ADC_SAMPLETIME_3CYCLES);
    g_power_battery->InitVREF();
    g_last_bridge_update_ms = HAL_GetTick();
}

void PowerTask(void* arg) {
    UNUSED(arg);

    while (true) {
        if (chassis != nullptr && g_power_battery != nullptr) {
            const uint32_t now_ms = HAL_GetTick();
            const float voltage = g_power_battery->GetBatteryVol();

            // Track the independent upper-board PowerTask heartbeat.
            // A dead upper board must not leave the last high referee limit
            // latched forever on the chassis controller.
            const uint32_t bridge_sequence = chassis->GetPowerBridgeSequence();
            if (!g_bridge_seen) {
                if (bridge_sequence != 0U) {
                    g_bridge_seen = true;
                    g_last_bridge_sequence = bridge_sequence;
                    g_last_bridge_update_ms = now_ms;
                    g_bridge_stale_active = false;
                }
            } else if (bridge_sequence != g_last_bridge_sequence) {
                g_last_bridge_sequence = bridge_sequence;
                g_last_bridge_update_ms = now_ms;
                g_bridge_stale_active = false;
            }

            const bool bridge_stale =
                static_cast<uint32_t>(now_ms - g_last_bridge_update_ms) >=
                atlas_power::kBridgeStaleTimeoutMs;
            if (bridge_stale && !g_bridge_stale_active) {
                chassis->UpdatePower(true,
                                     atlas_power::kBridgeFailsafePowerLimitW,
                                     voltage,
                                     atlas_power::kBridgeFailsafeBufferJoule,
                                     -1.0f, false);
                g_bridge_stale_active = true;
            }

            // Future-compatible local V*I path.  Today this callback is null,
            // so the estimator uses motor-model feed-forward + forwarded
            // referee Pr corrections.  Later only this adapter changes.
            float current_amp = 0.0f;
            const bool current_valid =
                g_current_reader != nullptr && g_current_reader(&current_amp) &&
                std::isfinite(current_amp);

            if (current_valid && std::isfinite(voltage) && voltage > 1.0f) {
                chassis->UpdateLocalPowerMeasurement(voltage, current_amp, true);
            } else {
                chassis->InvalidateLocalPowerMeasurement();
            }

            // Lower-board control loop only: MODEL (+ future V*I) + Referee
            // -> Kalman -> dynamic common scale.  It never directly sends CAN.
            chassis->PowerControlUpdate(voltage);
        }

        osDelay(kPowerControlPeriodMs);
    }
}


bool PowerControlIsBridgeStale() { return g_bridge_stale_active; }

uint32_t PowerControlGetBridgeAgeMs() {
    return static_cast<uint32_t>(HAL_GetTick() - g_last_bridge_update_ms);
}
