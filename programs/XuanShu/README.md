# XuanShu Dual-M6020 Gimbal Test App

This folder is an **application-only vehicle/program** for the existing
`UICRM-Test-RealTime-IO-R1` tree. It does not replace or modify BSP, drivers,
components, Atlas, CAN/UART refactors, or other vehicles.

## Hardware

- DJI Board Type-C
- CAN1
  - GM6020 Yaw: feedback ID `0x205`
  - GM6020 Pitch: feedback ID `0x206`
- DR16/DBUS: USART3
- Debug: USART6, 921600 baud
- Board IMU: BMI088 (gyro + accelerometer) + IMU-only AHRS + heater
- MiniPC auto-aim: USB, existing `communication::Host`

## Modes

Right switch:

- UP: KILL
- MID: MANUAL
- DOWN: AUTOAIM

After boot or DBUS reconnect the switch must pass through UP/KILL before either
active mode is allowed.

## First power-on

1. Keep right switch UP/KILL.
2. Keep `kUseConfiguredEncoderOffsets = false` in `gimbal_data.h`.
3. Verify USART6 shows both `0x205` and `0x206` online.
4. Move the board by hand and verify BMI088 IMU pitch/yaw signs.
5. Enter MID/MANUAL and make only small stick inputs.
6. Tune motor inversion, mechanical limits and PID in `gimbal_data.h`.
7. Only after manual control is stable, test DOWN/AUTOAIM.

## Auto-aim target convention

The default matches the existing Atlas/Minnie pattern: `TARGET_ANGLE` is a
world/IMU target and is passed to `Gimbal::TargetAbs()` + `UpdateIMU()`.
Set `kVisionTargetIsAbsoluteWorld = false` if your MiniPC sends angular error
instead.

With the current UICRM `Host` API, `NO_TARGET_FLAG` does not expose a separate
per-command timestamp to the application. For this test app, the vision sender
should send `TARGET_ANGLE.accuracy = 0` whenever no target is available. This
prevents a previously valid target from remaining accepted while the Host link
itself is still online.

## Tuning location

All vehicle-specific tuning lives in:

`gimbal/include/gimbal_data.h`

including motor IDs, offsets, inversion, mechanical limits, RC signs/rates,
IMU axis mapping/signs, auto-aim signs/convention, timeouts and all four M6020
PID loops.


## Magnetometer

This app intentionally does **not** initialize or use IST8310. AHRS runs in IMU-only mode using BMI088 gyroscope + accelerometer. Yaw therefore has normal gyro-integration drift; for a short-duration auto-aim gimbal test this is expected and avoids magnetic interference from motors and the robot chassis.
