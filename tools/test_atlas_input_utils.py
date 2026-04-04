#!/usr/bin/env python3
"""Regression checks for Atlas DBUS input utilities and configurable bindings."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RC = ROOT / "programs/Atlas/gimbal/src/rc.cpp"
CONTROL_CONFIG = ROOT / "programs/Atlas/gimbal/include/atlas_control_config.h"
ROBOT = ROOT / "programs/Atlas/gimbal/src/robot_state.cpp"
SHOOT = ROOT / "programs/Atlas/gimbal/src/shoot_task.cpp"
UTILS_H = ROOT / "boards/algorithm/include/utils.h"
UTILS_CPP = ROOT / "boards/algorithm/src/utils.cpp"
DBUS = ROOT / "boards/drivers/src/dbus.cpp"
REMOTE_INPUT = ROOT / "boards/drivers/include/remote_input.h"


def deadband(value: float, threshold: float) -> float:
    threshold = abs(threshold)
    return 0.0 if -threshold <= value <= threshold else value


def normalize(value: float, full_scale: float, threshold: float) -> float:
    if full_scale <= 0.0:
        return 0.0
    normalized = max(-1.0, min(1.0, value / full_scale))
    return deadband(normalized, threshold)


def main() -> None:
    assert deadband(0.009, 0.01) == 0.0
    assert deadband(-0.01, 0.01) == 0.0
    assert deadband(0.011, 0.01) == 0.011
    assert normalize(10.0, 660.0, 0.04) == 0.0
    assert normalize(30.0, 660.0, 0.04) > 0.0
    assert normalize(50.0, 660.0, 0.12) == 0.0
    assert normalize(100.0, 660.0, 0.12) > 0.0

    rc = RC.read_text(encoding="utf-8")
    config = CONTROL_CONFIG.read_text(encoding="utf-8")
    robot = ROBOT.read_text(encoding="utf-8")
    shoot = SHOOT.read_text(encoding="utf-8")
    utils_h = UTILS_H.read_text(encoding="utf-8")
    utils_cpp = UTILS_CPP.read_text(encoding="utf-8")
    dbus = DBUS.read_text(encoding="utf-8")
    remote_input = REMOTE_INPUT.read_text(encoding="utf-8")

    assert "bool KeyPressed(" not in rc
    assert "keyboard_code >> bit" not in rc
    assert "keyboard.bit." not in rc
    assert "enum class KeyboardKey" not in config
    assert remote_input.count("enum class KeyboardKey") == 1
    assert "using KeyboardKey = remote::KeyboardKey;" in config
    assert "const remote::RemoteInput input = dbus->GetInput();" in rc
    assert "input.KeyDown(" in rc
    assert "input.MouseButtonDown(" in rc
    assert "input.GetMouseAxis(" in rc
    assert "input.GetSwitch(" in rc
    assert "input.GetChannel(" in rc
    assert "dbus->ch" not in rc
    assert "dbus->mouse" not in rc
    assert "dbus->keyboard" not in rc

    for token in (
        "kMoveForwardKey", "kMoveBackwardKey", "kMoveLeftKey",
        "kMoveRightKey", "kCycleModeKey", "kManualModeKey",
        "kSpinModeKey", "kFollowModeKey", "kFrictionToggleKey",
        "kUnloadKey", "kUiClearKey", "kFireKey",
    ):
        assert token in config, token
        assert token in rc, token

    for token in (
        "kChassisForwardChannel", "kChassisLateralChannel",
        "kChassisTurnChannel", "kGimbalPitchChannel", "kGimbalYawChannel",
        "kChassisForwardChannelSign", "kChassisLateralChannelSign",
        "kChassisTurnChannelSign", "kGimbalPitchChannelSign",
        "kGimbalYawChannelSign", "kGimbalPitchMouseAxis",
        "kGimbalYawMouseAxis", "kGimbalPitchMouseSign",
        "kGimbalYawMouseSign",
    ):
        assert token in config, token
        assert token in rc, token

    assert "KeyboardBindingsUnique(kMovementKeys)" in config
    assert "KeyboardBindingsUnique(kFunctionKeys)" in config
    assert "KeyboardBindingGroupsDisjoint" in config
    assert "The physical KILL switch cannot be disabled" in config

    assert "kMouseDeadband = 0.01f" in rc
    assert "kStickDeadband = 0.04f" in rc
    assert "kTurnDeadband = 0.12f" in rc
    assert rc.count("normalize_symmetric(") >= 7
    assert "rc_key_process(" in rc
    assert "rc_key_process_events();" in rc

    assert "T apply_deadband(" in utils_h
    assert "float normalize_symmetric(" in utils_h
    assert "void reset(bool initial = false);" in utils_h
    assert "void BoolEdgeDetector::reset(bool initial)" in utils_cpp

    assert "bool Rising(" not in robot
    assert "BoolEdgeDetector cycle_edge" in robot
    assert "cycle_edge.posEdge()" in robot
    assert "bool Rising(" not in shoot
    assert "BoolEdgeDetector friction_edge" in shoot
    assert "friction_edge.posEdge()" in shoot

    assert "apply_deadband<int16_t>" in dbus
    assert dbus.index("Heartbeat();") > dbus.index("Read<true>")

    print("PASS: normalized deadband zeros values at and below threshold")
    print("PASS: remote_input.h owns shared keyboard/channel/switch/mouse input types")
    print("PASS: Atlas rc.cpp consumes one complete RemoteInput snapshot")
    print("PASS: channel selection and direction signs are centralized")
    print("PASS: mouse X/Y/Z are exposed through RemoteInput::GetMouseAxis")
    print("PASS: mouse/joystick/dial use shared normalize_symmetric utility")
    print("PASS: RCKEY remains the click/double-click/long-press engine")
    print("PASS: robot/shooter rising edges reuse BoolEdgeDetector")
    print("PASS: DBUS raw drift uses apply_deadband and invalid-length frames do not heartbeat")


if __name__ == "__main__":
    main()
