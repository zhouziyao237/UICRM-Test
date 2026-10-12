#pragma once

#include <cstdint>

#include "power_limit_lqr.h"

namespace atlas_power {

// Lower-board fail-safe if the independent upper-board power forward heartbeat
// disappears.  Atlas is an infantry chassis; keep zero buffer until valid
// bridged referee state resumes.
inline constexpr uint32_t kBridgeStaleTimeoutMs = 100U;
inline constexpr float kBridgeFailsafePowerLimitW = 45.0f;
inline constexpr float kBridgeFailsafeBufferJoule = 0.0f;

/**
 * Atlas vehicle-level power tuning profile.
 *
 * Competition rule values (Pl and official buffer Z) do NOT belong here; they
 * come from the referee system.  This file contains only vehicle/sensor/control
 * tuning that may legitimately differ between robots.
 */
inline control::LQRPowerLimit::Config MakeProfile() {
    control::LQRPowerLimit::Config cfg;

    // Control loop / prediction horizon.
    cfg.dt = 0.001f;
    cfg.prediction_horizon_s = 0.10f;

    // Start conservative before V/I and referee calibration is completed.
    cfg.referee_safety_margin = 0.90f;
    cfg.local_safety_margin = 0.90f;

    // Local chassis-bus sensor freshness.
    cfg.local_bus_timeout_s = 0.020f;

    // Kalman source order in the current car:
    //   1 kHz motor model -> continuous feed-forward observation
    //   ~10 Hz referee Pr -> real-power correction, fused once per new sample
    // Later, a registered V*I sensor becomes the preferred fast truth.
    cfg.kf_q_power = 0.15f;
    cfg.kf_q_rate = 0.02f;
    cfg.kf_r_model = 8.0f;
    cfg.kf_r_referee = 1.50f;
    cfg.kf_r_bus = 0.80f;

    // Dynamic common-scale controller.  These are the main per-vehicle dynamic
    // tuning values after the sensor and motor model have been calibrated.
    cfg.lqr_k_power = 0.90f;
    cfg.lqr_k_rate = 0.35f;
    cfg.lqr_k_buffer = 0.18f;
    cfg.lqr_buffer_target_joule = 20.0f;
    cfg.lqr_attack_per_second = 12.0f;
    cfg.lqr_release_per_second = 2.0f;

    // Online model correction rates.
    cfg.local_model_correction_alpha = 0.002f;
    cfg.referee_model_correction_alpha = 0.05f;

    // M3508/C620 default model. Re-identify these when motor/controller type,
    // transmission, or measured chassis branch changes materially.
    cfg.motor_model.k1 = 0.000931f;
    cfg.motor_model.k2 = 0.000455f;
    cfg.motor_model.k3 = 0.000006f;
    cfg.motor_model.k4 = 39.151850f;

    return cfg;
}

}  // namespace atlas_power
