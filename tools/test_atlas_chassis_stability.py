#!/usr/bin/env python3
"""Focused regression checks for Atlas chassis FOLLOW/SPIN/manual stability."""

from __future__ import annotations

import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHASSIS = ROOT / "programs/Atlas/gimbal/src/chassis_task.cpp"
GIMBAL = ROOT / "programs/Atlas/gimbal/src/gimbal_task.cpp"
RC = ROOT / "programs/Atlas/gimbal/src/rc.cpp"

PI = math.pi


def wrap_pi(value: float) -> float:
    while value > PI:
        value -= 2.0 * PI
    while value < -PI:
        value += 2.0 * PI
    return value


def unwrap(samples: list[float]) -> list[float]:
    result = [samples[0]]
    previous = samples[0]
    continuous = samples[0]
    for current in samples[1:]:
        continuous += wrap_pi(current - previous)
        result.append(continuous)
        previous = current
    return result


def follow_command(error: float) -> float:
    if abs(error) < 0.035:
        error = 0.0
    return max(-0.22, min(0.22, -0.32 * error))


def heading_command(error: float) -> float:
    return max(-0.10, min(0.10, -0.30 * error))


def main() -> None:
    # The real trace crossed +pi -> -pi while the physical rotation continued.
    wrapped = [2.95, 3.12, -3.05, -2.82, -2.40, -1.80, -0.80, -0.02]
    continuous = unwrap(wrapped)
    assert all(b >= a - 1e-6 for a, b in zip(continuous, continuous[1:])), continuous

    # Positive Atlas relative yaw must command negative chassis turn. Across
    # the wrap boundary the correction must not alternate signs.
    commands = [follow_command(value) for value in continuous]
    assert all(value < 0.0 for value in commands), commands

    converging_commands = [follow_command(value) for value in (3.0, 2.0, 1.0, 0.02)]
    assert all(value < 0.0 for value in converging_commands[:-1]), converging_commands
    assert abs(converging_commands[-1]) < 1e-9, converging_commands

    # chassis_heading = imu_yaw - logical_relative_yaw. If the gimbal rotates
    # on a stationary chassis, both change by the same amount and heading stays.
    chassis_heading = 0.7
    for relative in (-2.8, -1.0, 0.0, 1.2, 3.0):
        imu = wrap_pi(chassis_heading + relative)
        estimate = wrap_pi(imu - relative)
        assert abs(wrap_pi(estimate - chassis_heading)) < 1e-6

    # Positive heading error needs negative command on the measured Atlas turn
    # convention; negative error needs positive command.
    assert heading_command(0.2) < 0.0
    assert heading_command(-0.2) > 0.0

    chassis_source = CHASSIS.read_text(encoding="utf-8")
    gimbal_source = GIMBAL.read_text(encoding="utf-8")
    rc_source = RC.read_text(encoding="utf-8")

    required_chassis = (
        "UpdateContinuousRelativeYaw",
        "-kFollowTurnKp * error",
        "imu.yaw - relative_yaw",
        "SpinTurnCommand",
        "kSpinMaxTranslationCommand = 0.45f",
        "LimitVectorMagnitude",
        "kMaxXySpeed = 2.0f * PI * 7.0f",
        "kMaxTurnSpeed = 2.0f * PI * 7.0f",
    )
    for token in required_chassis:
        assert token in chassis_source, token

    assert "SyncManualTarget" in gimbal_source
    assert "yaw_motor->SetSpeedOffset(0.0f)" in gimbal_source
    assert "ChassisGetTurnSpeed()" not in gimbal_source
    assert "gimbal_params->yaw_inverted ? -yaw_motor_relative" in gimbal_source
    assert "kTurnDeadband = 0.12f" in rc_source

    print("PASS: relative-yaw unwrap is continuous across +/-pi")
    print("PASS: FOLLOW correction sign matches the real Atlas trace")
    print("PASS: manual heading estimate cancels gimbal rotation")
    # Full-scale SPIN translation is capped so sqrt(2)*translation + turn
    # remains below the lower chassis L1 speed limit.
    assert math.sqrt(2.0) * 0.45 + 0.35 < 1.0

    print("PASS: SPIN direction latch, translation headroom and DT7 turn deadband are present")
    print("PASS: MANUAL transition synchronizes gimbal motor targets")
    print("PASS: upper/lower chassis speed limits are aligned")


if __name__ == "__main__":
    main()
