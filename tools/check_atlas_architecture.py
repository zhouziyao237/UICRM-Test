#!/usr/bin/env python3
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
ATLAS = ROOT / "programs" / "Atlas" / "gimbal"
errors: list[str] = []

sources = list((ATLAS / "src").glob("*.cpp"))
headers = list((ATLAS / "include").glob("*.h"))

# Application behavior should stay procedural. Driver-library forward declarations
# in referee_task.h are allowed.
for path in sources:
    text = path.read_text()
    if re.search(r"\bclass\s+[A-Za-z_]", text):
        errors.append(f"{path}: application class found")
    if "enum class" in text:
        errors.append(f"{path}: enum class found")

robot_header = (ATLAS / "include" / "robot_state.h").read_text()
for token in ("extern volatile RemoteMode_e remote_mode;",
              "extern volatile RemoteMode_e last_remote_mode;",
              "void UpdateRobotMode("):
    if token not in robot_header:
        errors.append(f"robot_state.h missing {token}")

for path in sources:
    if path.name == "robot_state.cpp":
        continue
    text = path.read_text()
    if re.search(r"\bremote_mode\s*=(?!=)", text):
        errors.append(f"{path}: writes remote_mode outside robot_state.cpp")

for path in sources:
    text = path.read_text()
    for legacy in ("RobotStateInput", "RobotStateStep(",
                   "RCRegisterFireCallback", "RCRegisterUiCallback"):
        if legacy in text:
            errors.append(f"{path}: legacy complexity token {legacy}")
    if re.search(r"\bAtlasHardware\s*\(\s*\)", text):
        errors.append(f"{path}: legacy complexity token AtlasHardware()")

# Real-time hardening keeps the procedural architecture but freezes one published
# POD state frame per control iteration. This is deliberately not a controller class
# or hidden dispatch layer.
for name in ("chassis_task.cpp", "gimbal_task.cpp"):
    text = (ATLAS / "src" / name).read_text()
    if "RobotGetSnapshot()" not in text or "const RemoteMode_e mode" not in text:
        errors.append(f"{name}: control iteration does not freeze a robot-state snapshot")
    if "switch (remote_mode)" in text:
        errors.append(f"{name}: reads mutable remote_mode directly in control switch")

if errors:
    print("Atlas simple-architecture check FAILED")
    print("\n".join(errors))
    sys.exit(1)

print("Atlas simple-architecture check PASS")
print("One writer-owned remote_mode and one UpdateRobotMode()")
print("No Atlas application controller classes")
print("Control tasks use one published POD state snapshot per iteration")
print("Mode behavior still uses direct switch statements")
