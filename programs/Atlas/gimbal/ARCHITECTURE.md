# Atlas simple real-time control layout

Atlas keeps the original UICRM procedural application style. The real-time
hardening adds published POD snapshots, not controller classes or a new runtime
framework.

```text
DBUS -> rc.cpp writer -> RemoteControl_t published frame
                              |
                              v
                       UpdateRobotMode()
                              |
                     robot_state.cpp writer
                              |
                     RobotStateSnapshot_t
                       /       |       \
                ChassisTask GimbalTask ShootTask
```

## Mode control

There is still one writer-owned run-mode state in `robot_state.cpp`:

```cpp
volatile RemoteMode_e remote_mode;
volatile RemoteMode_e last_remote_mode;
```

`SysTask` reads DBUS/referee snapshots and calls:

```cpp
UpdateRobotMode(remote, referee, now_ms);
```

`robot_state.cpp` publishes a small `RobotStateSnapshot_t` after each state
update. A control task reads that frame once at the beginning of an iteration:

```cpp
const RobotStateSnapshot_t robot_state = RobotGetSnapshot();
const RemoteMode_e mode = robot_state.mode;
```

The rest of that iteration uses `mode` and an ordinary `switch (mode)`. This
prevents a higher-priority `SysTask` update from mixing transition logic from
one mode with output logic from another mode. There are still no
application-level mode controller classes, transition tables, polymorphic mode
objects, service containers or hidden dispatch layers.

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
| `rc.*` | DBUS decode, normalization, one-shot events and RC frame publication |
| `robot_state.*` | mode writer, HP=0 policy, reconnect re-arm and state publication |
| `imu_task.*` | BMI088/AHRS data |
| `gimbal_task.*` | Pitch/Yaw motors, homing and direct mode switch |
| `chassis_task.*` | Chassis command switch and power limiting |
| `shoot_task.*` | Flywheel/feeder state and shot actions |
| `referee_task.*` | Referee application snapshot |
| `ui_task.*` | Read-only display |

Low-level motor, CAN, UART, AHRS and GUI libraries remain C++ classes because
those are reusable UICRM drivers/components. Atlas application behavior itself
remains procedural.

## I/O ownership rules

- A DMA RX buffer stays owned by the UART driver. Deferred parsers consume the
  UART software stream through `ReadBuffered()`.
- A protocol transport chunk is not assumed to be one protocol frame; the
  parser retains an incomplete tail across chunks.
- bxCAN producers still call the original `CAN::Transmit()` API, but the CAN
  object owns HAL TX mailbox access and keeps only the latest pending value for
  each CAN ID.
- The chassis bridge retains registers `0x70/0x71`; the reserved 3-bit extension
  ID field is used as a transaction sequence so XY and turn/enable commit only
  when they belong to the same command generation.

## Control mapping configuration

DT7/DR16 channel signs, keyboard functions, switch bindings and mouse fire are
centralized in `include/atlas_control_config.h`. See
`ATLAS_CONTROL_CUSTOMIZATION.md` for the supported values and safety checks.
The DBUS keyboard bit mapping itself remains owned by
`boards/drivers/include/dbus_package.h`.
