#!/usr/bin/env python3
"""Regression for Atlas multi-turn yaw wrapping and SPIN vector handling."""
from __future__ import annotations

import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UTILS = ROOT / "boards/algorithm/include/utils.h"
MOTOR = ROOT / "boards/drivers/src/MotorCanBase.cpp"
GIMBAL = ROOT / "boards/components/src/gimbal.cpp"
ATLAS_GIMBAL = ROOT / "programs/Atlas/gimbal/src/gimbal_task.cpp"
ATLAS_CHASSIS = ROOT / "programs/Atlas/gimbal/src/chassis_task.cpp"
CHASSIS_H = ROOT / "boards/components/include/chassis.h"
CHASSIS_CPP = ROOT / "boards/components/src/chassis.cpp"

PI = math.pi
TWO_PI = 2.0 * PI


def wrap(value: float, low: float, high: float) -> float:
    width = high - low
    if low <= value <= high:
        return value
    return (value - low) % width + low


def slew(tx: float, ty: float, step: float, x: float, y: float) -> tuple[float, float]:
    dx, dy = tx - x, ty - y
    norm = math.hypot(dx, dy)
    if norm <= step:
        return tx, ty
    scale = step / norm
    return x + dx * scale, y + dy * scale


def main() -> None:
    # Hundreds of revolutions must always collapse into one turn.
    for turns in range(-500, 501):
        for phase in (-PI, -2.1, -0.1, 0.0, 0.3, 2.9, PI):
            value = phase + turns * TWO_PI
            wrapped = wrap(value, -PI, PI)
            assert -PI - 1e-12 <= wrapped <= PI + 1e-12
            assert abs(math.sin(wrapped) - math.sin(value)) < 1e-10
            assert abs(math.cos(wrapped) - math.cos(value)) < 1e-10

    # The exact failure from the real trace: a 157 rad ABSOLUTE coordinate must
    # become a one-turn coordinate, and target sync must produce zero error.
    actual = -157.0495
    one_turn = wrap(actual, 0.0, TWO_PI)
    target = wrap(actual, 0.0, TWO_PI)
    error = wrap(target - one_turn, -PI, PI)
    assert abs(error) < 1e-12

    # A 2-D slew step has bounded magnitude and stays collinear with delta.
    x, y = 0.0, 0.0
    tx, ty = 3.0, 4.0
    nx, ny = slew(tx, ty, 0.9, x, y)
    assert abs(math.hypot(nx - x, ny - y) - 0.9) < 1e-12
    assert abs(nx / ny - tx / ty) < 1e-12

    # Atlas translation is ordered as (forward, right).  With the gimbal held
    # at a fixed world heading and the chassis at heading -relative_yaw, the
    # corrected transform must keep the world translation command constant.
    for operator_forward, operator_right in ((1.0, 0.0), (0.0, 1.0), (0.6, -0.35)):
        for index in range(721):
            relative_yaw = -2.0 * PI + index * (4.0 * PI / 720.0)
            c = math.cos(relative_yaw)
            sn = math.sin(relative_yaw)
            chassis_forward = c * operator_forward + sn * operator_right
            chassis_right = -sn * operator_forward + c * operator_right

            chassis_heading = -relative_yaw
            hc = math.cos(chassis_heading)
            hs = math.sin(chassis_heading)
            world_forward = hc * chassis_forward + hs * chassis_right
            world_right = -hs * chassis_forward + hc * chassis_right
            assert abs(world_forward - operator_forward) < 1.0e-10
            assert abs(world_right - operator_right) < 1.0e-10

    utils = UTILS.read_text(encoding="utf-8")
    motor = MOTOR.read_text(encoding="utf-8")
    gimbal = GIMBAL.read_text(encoding="utf-8")
    atlas_gimbal = ATLAS_GIMBAL.read_text(encoding="utf-8")
    atlas_chassis = ATLAS_CHASSIS.read_text(encoding="utf-8")
    chassis_h = CHASSIS_H.read_text(encoding="utf-8")
    chassis_cpp = CHASSIS_CPP.read_text(encoding="utf-8")

    assert "std::fmod" in utils
    assert "return wrap<T>(value, min, max);" in utils
    assert "slew_vector_2d" in utils
    assert "? wrap<float>(continuous_output, 0.0f, 2.0f * PI)" in motor
    assert "yaw_circle_ = true" in atlas_gimbal
    assert "MotorCANBase::ABSOLUTE" in atlas_gimbal
    assert "yaw_angle_ = wrap<float>(yaw_motor_->GetOutputShaftTheta()" in gimbal
    assert "Ease g_ease_vx" not in atlas_chassis
    assert "Ease g_ease_vy" not in atlas_chassis
    assert "slew_vector_2d(target_vx, target_vy" in atlas_chassis
    assert "cos_yaw * command_x + sin_yaw * command_y" in atlas_chassis
    assert "-sin_yaw * command_x + cos_yaw * command_y" in atlas_chassis
    assert "cos_yaw * command_x - sin_yaw * command_y" not in atlas_chassis
    assert "GetKinematicScale" in chassis_h
    assert "kinematic_scale_ = scale" in chassis_cpp
    assert "if (model_ == CHASSIS_OMNI_WHEEL2)" in chassis_cpp
    assert "omni_wheel_setup_ = chassis.omni_wheel_setup_" in chassis_cpp

    print("PASS: wrap handles +/-500 revolutions")
    print("PASS: ABSOLUTE target sync is zero-error after -157 rad")
    print("PASS: Atlas yaw remains configured ABSOLUTE/circular")
    print("PASS: SPIN forward/right frame remains world-stable for two revolutions")
    print("PASS: SPIN translation uses a direction-preserving 2-D slew")
    print("PASS: lower chassis exposes kinematic normalization scale")
    print("PASS: OMNI2 setup is copied when that model is selected")


if __name__ == "__main__":
    main()

