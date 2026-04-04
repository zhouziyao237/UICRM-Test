#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "power_limit_lqr.h"

namespace {
bool Near(float a, float b, float tolerance = 1e-3f) {
    return std::fabs(a - b) <= tolerance;
}
}

int main() {
    using control::LQRPowerLimit;
    using control::SeasonRuleManager;

    // Official 2026 accounting: 158 W at a 100 W limit for 100 ms costs 5.8 J.
    SeasonRuleManager rule(SeasonRuleManager::SEASON_2026);
    rule.SyncBuffer(60.0f);
    rule.CalcBufferConsume(158.0f, 100.0f, 0.1f);
    assert(Near(rule.GetBuffer(), 54.2f, 1e-4f));

    // Below-limit operation restores energy using the same expression.
    rule.CalcBufferConsume(50.0f, 100.0f, 0.1f);
    assert(Near(rule.GetBuffer(), 59.2f, 1e-4f));
    rule.CalcBufferConsume(0.0f, 100.0f, 0.1f);
    assert(Near(rule.GetBuffer(), 60.0f, 1e-4f));

    LQRPowerLimit::Config config;
    config.dt = 0.001f;
    config.referee_safety_margin = 0.90f;
    config.local_safety_margin = 0.90f;
    LQRPowerLimit limiter(config);

    float omega[4] = {100.0f, -80.0f, 75.0f, -95.0f};
    int16_t current[4] = {30000, -26000, 28000, -30000};
    const int16_t original[4] = {current[0], current[1], current[2], current[3]};

    limiter.UpdatePowerState(true, 100.0f, 24.0f, 60.0f, -1.0f, false);
    limiter.UpdateMotorData(omega, current, 4);
    limiter.ApplyLimit(current, omega, 4);

    assert(!limiter.IsRefereeSource());
    assert(limiter.GetCurrentScale() >= 0.0f && limiter.GetCurrentScale() <= 1.0f);
    assert(limiter.GetLimitedPowerEstimate() <= 90.05f);
    assert(limiter.GetCurrentScale() < 1.0f);

    // All wheels must retain one common current scale and their signs.
    for (int i = 0; i < 4; ++i) {
        if (original[i] != 0) {
            const float ratio = static_cast<float>(current[i]) /
                                static_cast<float>(original[i]);
            assert(std::fabs(ratio - limiter.GetCurrentScale()) < 0.002f);
            assert((current[i] > 0) == (original[i] > 0));
        }
    }

    // Referee data is authoritative and synchronises buffer directly.
    limiter.UpdatePowerState(true, 100.0f, 24.0f, 12.0f, 80.0f, true);
    assert(limiter.IsRefereeSource());
    assert(Near(limiter.GetBuffer(), 12.0f));
    assert(Near(limiter.GetMeasuredPower(), 80.0f));

    // One 10 Hz referee measurement may be read repeatedly by the 1 kHz motor
    // loop. It must update the model once, not integrate the stale value 100x.
    int16_t correction_command[4] = {22000, -22000, 22000, -22000};
    limiter.UpdatePowerState(true, 100.0f, 24.0f, 60.0f, 105.0f, true);
    limiter.UpdateMotorData(omega, correction_command, 4);
    limiter.ApplyLimit(correction_command, omega, 4);
    const float correction_after_new_sample = limiter.GetModelCorrection();

    for (int step = 0; step < 100; ++step) {
        int16_t command[4] = {22000, -22000, 22000, -22000};
        limiter.UpdatePowerState(true, 100.0f, 24.0f, 60.0f, 105.0f, true);
        limiter.UpdateMotorData(omega, command, 4);
        limiter.ApplyLimit(command, omega, 4);
    }
    assert(Near(limiter.GetModelCorrection(), correction_after_new_sample,
                1e-6f));

    // A distinct measurement advances the slow correction, within hard bounds.
    limiter.UpdatePowerState(true, 100.0f, 24.0f, 60.0f, 106.0f, true);
    assert(!Near(limiter.GetModelCorrection(), correction_after_new_sample,
                 1e-6f));
    assert(limiter.GetModelCorrection() >= 0.60f);
    assert(limiter.GetModelCorrection() <= 1.80f);
    assert(limiter.GetLimitedPowerEstimate() <= 90.05f);

    // Local fallback uses the same 100 ms rule and restores buffer below Pl.
    limiter.UpdatePowerState(true, 100.0f, 24.0f, 30.0f, -1.0f, false);
    for (int step = 0; step < 100; ++step) {
        int16_t idle[4] = {0, 0, 0, 0};
        float idle_omega[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        limiter.UpdatePowerState(true, 100.0f, 24.0f, 30.0f, -1.0f, false);
        limiter.UpdateMotorData(idle_omega, idle, 4);
        limiter.ApplyLimit(idle, idle_omega, 4);
    }
    assert(limiter.GetBuffer() > 30.0f);
    assert(limiter.GetBuffer() <= 60.0f);

    // Invalid/disabled limit fails safe to zero output.
    int16_t invalid[4] = {10000, 10000, 10000, 10000};
    limiter.UpdatePowerState(true, 0.0f, 24.0f, 60.0f, -1.0f, false);
    limiter.UpdateMotorData(omega, invalid, 4);
    limiter.ApplyLimit(invalid, omega, 4);
    for (int i = 0; i < 4; ++i) assert(invalid[i] == 0);

    std::puts("RM 2026 power rule tests: PASS");
    return 0;
}
