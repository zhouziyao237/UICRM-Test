#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
config = (ROOT / "programs/Atlas/gimbal/include/atlas_config.h").read_text()
source = (ROOT / "programs/Atlas/gimbal/src/chassis_task.cpp").read_text()
bridge = (ROOT / "boards/components/src/chassis.cpp").read_text()
algorithm = (ROOT / "boards/algorithm/src/power_limit_lqr.cpp").read_text()

checks = {
    "no-referee bench source is an explicit compile-time test macro": (
        "#define ATLAS_ENABLE_NO_REFEREE_POWER_TEST 1" in config
        and "#if ATLAS_ENABLE_NO_REFEREE_POWER_TEST" in config
        and "kLocalChassisPowerLimitW = 100U" in config
        and "kLocalChassisBufferJoule = 60.0f" in config
        and "#if ATLAS_ENABLE_NO_REFEREE_POWER_TEST" in source
        and "local_test_active = true" in source
    ),
    "formal no-referee path starts from 45 W / 0 J fail-safe": (
        "uint8_t max_watt = kInvalidRefereePowerLimitW" in source
        and "float buffer_joule = kInvalidRefereeBufferJoule" in source
        and "bool referee_failsafe = true" in source
    ),
    "online invalid referee data uses 45 W / 0 J fail-safe": (
        "kInvalidRefereePowerLimitW = 45U" in config
        and "kInvalidRefereeBufferJoule = 0.0f" in config
        and "referee_failsafe = true" in source
    ),
    "valid limit/buffer does not depend on measured-power validity": (
        "referee_limit_valid =" in source
        and "std::isfinite(referee.chassis_power)" in source
        and source.index("if (referee_limit_valid)")
        < source.index("std::isfinite(referee.chassis_power)")
    ),
    "referee data is forwarded at 100 ms cadence": (
        "kPowerUpdatePeriodMs = 100U" in source
        and "HAL_GetTick()" in source
    ),
    "CAN bridge sends source, limit, voltage, measured power and buffer": all(
        token in bridge
        for token in [
            "data_eight_uint8.data[1] = max_watt",
            "data_eight_uint8.data[3] = referee_online ? 1U : 0U",
            "data_two_float.data[1] = current_voltage",
            "data_two_float.data[0] = measured_power",
            "data_two_float.data[1] = buffer_joule",
        ]
    ),
    "old 1.5x buffer multiplier is absent from the 2026 limiter": (
        "buffer_scale" not in algorithm and "1.5f" not in algorithm
    ),
    "stale 10 Hz referee sample is not integrated at 1 kHz": (
        "new_correction_sample" in algorithm
        and "last_correction_sample_power_" in algorithm
    ),
}

failed = [name for name, passed in checks.items() if not passed]
for name, passed in checks.items():
    print(("PASS" if passed else "FAIL") + ": " + name)
if failed:
    raise SystemExit(1)
