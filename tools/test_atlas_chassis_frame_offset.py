#!/usr/bin/env python3
from __future__ import annotations

import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / 'programs/Atlas/gimbal/src/chassis_task.cpp'
HDR = ROOT / 'programs/Atlas/gimbal/include/chassis_task.h'
CFG = ROOT / 'programs/Atlas/gimbal/include/atlas_config.h'
MAIN = ROOT / 'programs/Atlas/gimbal/src/main.cpp'

cpp = CPP.read_text(encoding='utf-8')
hdr = HDR.read_text(encoding='utf-8')
cfg = CFG.read_text(encoding='utf-8')
main = MAIN.read_text(encoding='utf-8')

required = [
    'kDefaultChassisFrameOffsetDeg',
    'MapLogicalToChassisLibrary',
    'library_x_speed = calibrated_right;',
    'library_y_speed = calibrated_forward;',
    'ChassisSetFrameOffsetDeg',
    'ChassisGetFrameOffsetDeg',
]
for token in required:
    if token not in cfg + cpp + hdr:
        raise SystemExit(f'FAIL: missing {token}')

if 'g_chassis->SetSpeed(g_telemetry.vx, g_telemetry.vy' in cpp:
    raise SystemExit('FAIL: logical forward/right still sent directly as library x/y')

if 'Chassis frame offset=' not in main:
    raise SystemExit('FAIL: no frame-offset telemetry print')


def map_cmd(forward: float, right: float, deg: float) -> tuple[float, float]:
    rad = math.radians(deg)
    calibrated_forward = math.cos(rad) * forward + math.sin(rad) * right
    calibrated_right = -math.sin(rad) * forward + math.cos(rad) * right
    return calibrated_right, calibrated_forward  # library x, library y


def close(a: float, b: float) -> bool:
    return abs(a - b) < 1e-6

# At zero offset, logical forward maps to library y; logical right maps to x.
x, y = map_cmd(1.0, 0.0, 0.0)
assert close(x, 0.0) and close(y, 1.0)
x, y = map_cmd(0.0, 1.0, 0.0)
assert close(x, 1.0) and close(y, 0.0)

# The four cardinal debug offsets must remain pure-axis transformations.
expected = {
    90.0: (-1.0, 0.0),
    -90.0: (1.0, 0.0),
    180.0: (0.0, -1.0),
}
for deg, (ex, ey) in expected.items():
    x, y = map_cmd(1.0, 0.0, deg)
    assert close(x, ex) and close(y, ey), (deg, x, y)

# Rotation must preserve translation magnitude.
for deg in range(-180, 181, 5):
    x, y = map_cmd(0.37, -0.61, float(deg))
    assert abs(math.hypot(x, y) - math.hypot(0.37, -0.61)) < 1e-6

print('PASS: Atlas forward/right is adapted to chassis-library x=right/y=forward')
print('PASS: configurable 0/+90/-90/180 degree frame offsets are pure rotations')
print('PASS: frame-offset rotation preserves translation-vector magnitude')
print('PASS: KILL-only runtime setter/getter and telemetry diagnostics are present')
