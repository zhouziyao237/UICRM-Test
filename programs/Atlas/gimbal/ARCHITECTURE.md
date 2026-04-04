# Atlas simple control layout

Atlas now uses a deliberately direct application structure.

```text
DBUS -> rc.cpp -> RemoteControl_t
                     |
                     v
              UpdateRobotMode()
                     |
              remote_mode global
            /        |        \
     ChassisTask  GimbalTask  ShootTask
```

## Mode control

There is one run-mode variable only:

```cpp
volatile RemoteMode_e remote_mode;
volatile RemoteMode_e last_remote_mode;
```

`SysTask` reads DBUS/referee snapshots and calls:

```cpp
UpdateRobotMode(remote, referee, now_ms);
```

The chassis and gimbal tasks use an ordinary `switch (remote_mode)`.
There are no application-level mode controller classes, transition tables,
polymorphic mode objects or hidden dispatch layers.

## Adding a new mode

1. Add one enum value in `include/robot_state.h`.
2. Add it to `NextRemoteMode()` in `src/robot_state.cpp` when it should appear in the G-key/DT7 cycle.
3. Add a `case` in `src/chassis_task.cpp`.
4. Add a `case` in `src/gimbal_task.cpp` when the gimbal behavior differs.
5. Optionally add the display string in `ui_task.cpp` and MiniPC mapping in `minipc_task.cpp`.

No class needs to be created.

## File responsibilities

| File | Responsibility |
|---|---|
| `sys.*` | Named hardware pointers and the 10 ms mode-update task |
| `rc.*` | DBUS decode, normalization and one-shot operator events |
| `robot_state.*` | `remote_mode`, HP=0 latch and reconnect re-arm |
| `imu_task.*` | BMI088/AHRS data |
| `gimbal_task.*` | Pitch/Yaw motors, homing and mode switch |
| `chassis_task.*` | Chassis command switch and power limiting |
| `shoot_task.*` | Flywheel/feeder state and shot actions |
| `referee_task.*` | Referee snapshot |
| `ui_task.*` | Read-only display |

The low-level motor, CAN, UART, AHRS and GUI libraries remain C++ classes because
those are existing reusable drivers. Atlas application behavior itself is
procedural.

## Control mapping configuration

DT7/DR16 channel signs, keyboard functions, switch bindings and mouse fire are
centralized in `include/atlas_control_config.h`. See
`ATLAS_CONTROL_CUSTOMIZATION.md` for the supported values and safety checks.
The DBUS keyboard bit mapping itself remains owned by
`boards/drivers/include/dbus_package.h`.
