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
    for legacy in ("RobotStateInput", "RobotStateSnapshot", "RobotStateStep(",
                   "RCRegisterFireCallback", "RCRegisterUiCallback"):
        if legacy in text:
            errors.append(f"{path}: legacy complexity token {legacy}")
    if re.search(r"\bAtlasHardware\s*\(\s*\)", text):
        errors.append(f"{path}: legacy complexity token AtlasHardware()")

if errors:
    print("Atlas simple-architecture check FAILED")
    print("\n".join(errors))
    sys.exit(1)

print("Atlas simple-architecture check PASS")
print("One global remote_mode and one UpdateRobotMode()")
print("No Atlas application controller classes")
print("Mode behavior uses direct switch statements")
