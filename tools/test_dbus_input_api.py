#!/usr/bin/env python3
"""Static regression checks for the shared DBUS/VTM RemoteInput API."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
COMMON = ROOT / "boards/drivers/include/remote_input.h"
DBUS_PACKAGE = ROOT / "boards/drivers/include/dbus_package.h"
VTM_PACKAGE = ROOT / "boards/drivers/include/vtm_package.h"
DBUS_H = ROOT / "boards/drivers/include/dbus.h"
DBUS_CPP = ROOT / "boards/drivers/src/dbus.cpp"
VTM_H = ROOT / "boards/drivers/include/vtm.h"
VTM_CPP = ROOT / "boards/drivers/src/vtm.cpp"
RC = ROOT / "programs/Atlas/gimbal/src/rc.cpp"


def main() -> None:
    common = COMMON.read_text(encoding="utf-8")
    dbus_package = DBUS_PACKAGE.read_text(encoding="utf-8")
    vtm_package = VTM_PACKAGE.read_text(encoding="utf-8")
    dbus_h = DBUS_H.read_text(encoding="utf-8")
    dbus_cpp = DBUS_CPP.read_text(encoding="utf-8")
    vtm_h = VTM_H.read_text(encoding="utf-8")
    vtm_cpp = VTM_CPP.read_text(encoding="utf-8")
    rc = RC.read_text(encoding="utf-8")

    for token in (
        "enum class RemoteSource", "enum class Channel",
        "enum class SwitchSide", "enum class KeyboardKey",
        "enum class MouseAxis", "enum class MouseButton",
        "enum class GearPosition", "enum class AuxButton",
        "struct RemoteInput", "GetChannel(Channel channel)",
        "HasSwitch(SwitchSide side)", "KeyDown(KeyboardKey key)",
        "GetMouseAxis(MouseAxis axis)",
        "MouseButtonDown(MouseButton button)",
        "AuxButtonDown(AuxButton button)",
    ):
        assert token in common, token

    assert common.count("enum class KeyboardKey") == 1
    assert "enum class KeyboardKey" not in dbus_package
    assert "enum class KeyboardKey" not in vtm_package
    assert "using vtm_keyboard_t = keyboard_t;" in vtm_package

    assert "RemoteInput GetInput() const;" in dbus_h
    assert "RemoteInput DBUS::GetInput() const" in dbus_cpp
    assert "next.source = RemoteSource::DBUS;" in dbus_cpp
    assert "next.mouse_middle = false;" in dbus_cpp

    assert "RemoteInput GetInput() const;" in vtm_h
    assert "RemoteInput VTM::GetInput() const" in vtm_cpp
    assert "next.source = RemoteSource::VTM;" in vtm_cpp
    assert "next.left_y" in vtm_cpp and "repr->ch2" in vtm_cpp
    assert "next.left_x" in vtm_cpp and "repr->ch3" in vtm_cpp
    assert "next.mouse_middle = repr->mouse.m != 0U;" in vtm_cpp
    assert "next.trigger =" in vtm_cpp
    assert "next.gear = GearPosition::" in vtm_cpp

    assert "const remote::RemoteInput input = dbus->GetInput();" in rc
    assert "input.HasSwitch(binding.side)" in rc
    assert "remote::DBUSInput" not in rc

    print("PASS: RemoteInput is shared by DBUS and VTM")
    print("PASS: protocol packet layouts remain in their own package headers")
    print("PASS: VTM left-axis order, trigger, gear, extra buttons and mouse middle are decoded")
    print("PASS: Atlas consumes the shared semantic frame")


if __name__ == "__main__":
    main()
