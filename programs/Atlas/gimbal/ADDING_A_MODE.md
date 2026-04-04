# Add an Atlas mode

Example: add `REMOTE_MODE_DEFENSE`.

## 1. Add the enum

`include/robot_state.h`

```cpp
enum RemoteMode_e : uint8_t {
    REMOTE_MODE_KILL = 0,
    REMOTE_MODE_MANUAL,
    REMOTE_MODE_SPIN,
    REMOTE_MODE_FOLLOW,
    REMOTE_MODE_DEFENSE,
};
```

## 2. Add it to mode cycling

`src/robot_state.cpp`

```cpp
RemoteMode_e NextRemoteMode(RemoteMode_e mode) {
    switch (mode) {
        case REMOTE_MODE_MANUAL:  return REMOTE_MODE_SPIN;
        case REMOTE_MODE_SPIN:    return REMOTE_MODE_FOLLOW;
        case REMOTE_MODE_FOLLOW:  return REMOTE_MODE_DEFENSE;
        case REMOTE_MODE_DEFENSE:
        default:                  return REMOTE_MODE_MANUAL;
    }
}
```

## 3. Add chassis behavior

`src/chassis_task.cpp`

```cpp
case REMOTE_MODE_DEFENSE:
    target_vx = field_x;
    target_vy = field_y;
    target_vt = 0.3f;
    break;
```

## 4. Add gimbal behavior only when needed

`src/gimbal_task.cpp`

```cpp
case REMOTE_MODE_DEFENSE:
    UpdateWorldControl();
    break;
```

That is the complete control path. No mode class, inheritance, registry or
callback chain is required.
