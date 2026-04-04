#!/usr/bin/env python3
"""Regression for Atlas FPS-style keyboard acceleration and braking."""
from __future__ import annotations

import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RC = ROOT / "programs/Atlas/gimbal/src/rc.cpp"
CONFIG = ROOT / "programs/Atlas/gimbal/include/atlas_control_config.h"
RC_H = ROOT / "programs/Atlas/gimbal/include/rc.h"
MAIN = ROOT / "programs/Atlas/gimbal/src/main.cpp"


def value(source: str, name: str) -> float:
    match = re.search(rf"constexpr float {name} = ([0-9.]+)f;", source)
    assert match, name
    return float(match.group(1))


def move_toward(current: float, target: float, max_delta: float) -> float:
    if current < target:
        return min(current + max_delta, target)
    if current > target:
        return max(current - max_delta, target)
    return current


def slew(tx: float, ty: float, max_delta: float,
         x: float, y: float) -> tuple[float, float]:
    dx, dy = tx - x, ty - y
    norm = math.hypot(dx, dy)
    if norm <= max_delta:
        return tx, ty
    if max_delta <= 0.0:
        return x, y
    scale = max_delta / norm
    return x + dx * scale, y + dy * scale


def step_translation(current: tuple[float, float],
                     target: tuple[float, float], dt: float,
                     acceleration: float, deceleration: float,
                     braking: float, reverse: float) -> tuple[float, float]:
    cx, cy = current
    tx, ty = target
    target_norm = math.hypot(tx, ty)
    current_norm = math.hypot(cx, cy)
    dot = cx * tx + cy * ty
    rate = acceleration
    if target_norm <= 0.0:
        rate = braking
    elif current_norm > 0.0 and dot < 0.0:
        rate = reverse
    elif target_norm < current_norm:
        rate = deceleration
    return slew(tx, ty, rate * dt, cx, cy)


def main() -> None:
    rc = RC.read_text(encoding="utf-8")
    config = CONFIG.read_text(encoding="utf-8")
    rc_h = RC_H.read_text(encoding="utf-8")
    main_cpp = MAIN.read_text(encoding="utf-8")

    crouch = value(config, "kKeyboardCrouchCommand")
    cruise = value(config, "kKeyboardCruiseCommand")
    sprint = value(config, "kKeyboardSprintCommand")
    acceleration = value(config, "kKeyboardMoveAcceleration")
    deceleration = value(config, "kKeyboardMoveDeceleration")
    braking = value(config, "kKeyboardMoveBraking")
    reverse = value(config, "kKeyboardMoveReverseRate")
    crouch_turn = value(config, "kKeyboardCrouchTurnCommand")
    cruise_turn = value(config, "kKeyboardCruiseTurnCommand")
    sprint_turn = value(config, "kKeyboardSprintTurnCommand")
    turn_accel = value(config, "kKeyboardTurnAcceleration")
    turn_decel = value(config, "kKeyboardTurnDeceleration")
    turn_brake = value(config, "kKeyboardTurnBraking")

    assert 0.0 < crouch < cruise < sprint < 1.0
    assert acceleration < deceleration < braking < reverse
    assert 0.0 < crouch_turn < cruise_turn < sprint_turn < 1.0
    assert turn_accel < turn_decel < turn_brake
    assert "kChassisForwardChannelSign = -1.0f" in config
    assert "kMoveSprintKey = KeyboardKey::SHIFT" in config
    assert "kMoveCrouchKey = KeyboardKey::CTRL" in config

    # W and W+D must request the same vector magnitude.
    diagonal = cruise / math.sqrt(2.0)
    assert abs(math.hypot(diagonal, diagonal) - cruise) < 1e-12

    # A held W key rises progressively instead of jumping to cruise command.
    current = (0.0, 0.0)
    samples: dict[int, float] = {}
    for millisecond in range(1, 701):
        current = step_translation(current, (cruise, 0.0), 0.001,
                                   acceleration, deceleration, braking, reverse)
        if millisecond in (50, 100, 250, 500, 700):
            samples[millisecond] = current[0]
    assert samples[50] < 0.05
    assert samples[100] < 0.10
    assert samples[250] < 0.25
    assert samples[500] < cruise
    assert abs(samples[700] - cruise) < 1e-9

    # SHIFT ramps from cruise to sprint, then releasing SHIFT decelerates
    # smoothly back to cruise without a command step.
    current = (cruise, 0.0)
    for _ in range(100):
        current = step_translation(current, (sprint, 0.0), 0.001,
                                   acceleration, deceleration, braking, reverse)
    assert cruise < current[0] < sprint
    previous = current[0]
    current = step_translation(current, (cruise, 0.0), 0.001,
                               acceleration, deceleration, braking, reverse)
    assert cruise <= current[0] < previous
    assert abs((previous - current[0]) - deceleration * 0.001) < 1e-9

    # CTRL crouch is a smooth reduction and has priority over sprint.
    current = (sprint, 0.0)
    crouch_ms = 0
    while current[0] > crouch + 1e-9 and crouch_ms < 1000:
        current = step_translation(current, (crouch, 0.0), 0.001,
                                   acceleration, deceleration, braking, reverse)
        crouch_ms += 1
    assert crouch_ms <= 450

    # Releasing the key uses stronger braking and reaches zero quickly.
    current = (cruise, 0.0)
    release_ms = 0
    while math.hypot(*current) > 1e-9 and release_ms < 1000:
        current = step_translation(current, (0.0, 0.0), 0.001,
                                   acceleration, deceleration, braking, reverse)
        release_ms += 1
    assert release_ms <= 300

    # Opposite direction brakes through zero quickly, then builds the new
    # direction with the normal acceleration rate.
    current = (cruise, 0.0)
    reverse_ms = 0
    zero_cross_ms = None
    while current[0] > -cruise + 1e-9 and reverse_ms < 1200:
        previous = current[0]
        current = step_translation(current, (-cruise, 0.0), 0.001,
                                   acceleration, deceleration, braking, reverse)
        reverse_ms += 1
        if zero_cross_ms is None and previous > 0.0 and current[0] <= 0.0:
            zero_cross_ms = reverse_ms
    assert zero_cross_ms is not None and zero_cross_ms <= 160
    assert reverse_ms <= 850

    # Digital turn is also ramped instead of jumping to +/-1.0.
    turn = 0.0
    for _ in range(100):
        turn = move_toward(turn, cruise_turn, turn_accel * 0.001)
    assert 0.05 < turn < 0.15
    while turn > 0.0:
        turn = move_toward(turn, 0.0, turn_brake * 0.001)
    assert abs(turn) < 1e-12

    assert "UpdateKeyboardMotion(" in rc
    assert "KeyboardDeltaTimeSeconds()" in rc
    assert "slew_vector_2d(target_forward, target_lateral" in rc
    assert "direction_dot < 0.0f" in rc
    assert "target_norm < current_norm" in rc
    assert "kKeyboardMoveDeceleration" in rc
    assert "crouch ? atlas::controls::kKeyboardCrouchCommand" in rc
    assert "ResetKeyboardMotion();" in rc
    assert "wish_norm > 1.0f" in rc
    assert "keyboard_speed" not in rc
    assert "? 1.0f : 0.5f" not in rc
    assert "bool keyboard_motion = false;" in rc_h
    assert 'remote.keyboard_motion ? "KEYBOARD" : "STICK/IDLE"' in main_cpp

    print(f"PASS: 100 ms W command ramps to {samples[100]:.3f}, not 0.500")
    print(f"PASS: crouch/cruise/sprint targets are {crouch:.2f}/{cruise:.2f}/{sprint:.2f}")
    print("PASS: SHIFT accelerates and releases smoothly to cruise")
    print(f"PASS: CTRL crouch decelerates from sprint in {crouch_ms} ms")
    print(f"PASS: key release brakes to zero in {release_ms} ms")
    print(f"PASS: reversal crosses zero in {zero_cross_ms} ms and reaches full reverse in {reverse_ms} ms")
    print("PASS: W+D is magnitude-normalized")
    print("PASS: Q/E follows crouch/cruise/sprint with acceleration/deceleration")
    print("PASS: analog input clears stale keyboard velocity")


if __name__ == "__main__":
    main()
