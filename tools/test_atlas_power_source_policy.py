#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
config = (ROOT / "programs/Atlas/gimbal/include/atlas_config.h").read_text()
forward = (ROOT / "programs/Atlas/gimbal/src/power_task.cpp").read_text()
bridge = (ROOT / "boards/components/src/chassis.cpp").read_text()
algorithm = (ROOT / "boards/algorithm/src/power_limit_lqr.cpp").read_text()
protocol = (ROOT / "boards/drivers/src/protocol.cpp").read_text()

checks = {
    "no-referee bench source is opt-in and disabled by default": (
        "#define ATLAS_ENABLE_NO_REFEREE_POWER_TEST 0" in config
        and "#if ATLAS_ENABLE_NO_REFEREE_POWER_TEST" in forward
        and "kLocalChassisPowerLimitW = 100U" in config
        and "kLocalChassisBufferJoule = 60.0f" in config
    ),
    "formal no-referee path is 45 W / 0 J fail-safe": (
        "max_watt = kInvalidRefereePowerLimitW" in forward
        and "buffer_joule = kInvalidRefereeBufferJoule" in forward
        and "failsafe = true" in forward
        and "kInvalidRefereePowerLimitW = 45U" in config
        and "kInvalidRefereeBufferJoule = 0.0f" in config
    ),
    "valid referee limit does not depend on measured-power validity": (
        "state.referee_limit_valid =" in forward
        and "std::isfinite(referee.chassis_power)" in forward
        and forward.index("if (state.referee_limit_valid)")
        < forward.index("std::isfinite(referee.chassis_power)")
    ),
    "upper power forwarder polls at 1 ms and uses a 20 ms state heartbeat": (
        "kPowerForwardPollMs = 1U" in forward
        and "kPowerLimitHeartbeatMs = 20U" in forward
    ),
    "actual referee power frames carry a source sequence": (
        "++power_heat_sequence_" in protocol
        and "power_sequence != g_last_forwarded_power_sequence" in forward
    ),
    "0x72 and 0x73 are independently forwardable": (
        "UpdatePowerLimitState" in bridge
        and "UpdateCurrentPower" in bridge
    ),
    "old 1.5x buffer multiplier is absent from the 2026 limiter": (
        "buffer_scale" not in algorithm and "1.5f" not in algorithm
    ),
    "stale referee sample is fused once per forwarded sample": (
        "referee_sample_sequence_ != last_fused_referee_sequence_" in algorithm
        and "kf_r_referee" in algorithm
    ),
}

failed = [name for name, passed in checks.items() if not passed]
for name, passed in checks.items():
    print(("PASS" if passed else "FAIL") + ": " + name)
if failed:
    raise SystemExit(1)
