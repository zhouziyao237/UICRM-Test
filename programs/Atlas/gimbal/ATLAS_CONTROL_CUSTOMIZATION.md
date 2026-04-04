# Atlas 输入与功能绑定配置

所有可修改映射集中在：

```text
programs/Atlas/gimbal/include/atlas_control_config.h
```

底层 DBUS 统一接口位于：

```text
boards/drivers/include/dbus_package.h
boards/drivers/include/dbus.h
```

Atlas 每周期只读取一次完整输入帧：

```cpp
const remote::RemoteInput input = dbus->GetInput();
```

应用层不再直接读取 `ch0`、`mouse.x`、`keyboard.code`、`swl` 等原始成员。

## 1. 遥控通道绑定

可选通道：

```cpp
remote::Channel::RIGHT_X
remote::Channel::RIGHT_Y
remote::Channel::LEFT_X
remote::Channel::LEFT_Y
remote::Channel::DIAL
```

默认配置：

```cpp
constexpr Channel kChassisForwardChannel = Channel::RIGHT_Y;
constexpr Channel kChassisLateralChannel = Channel::RIGHT_X;
constexpr Channel kChassisTurnChannel = Channel::DIAL;
constexpr Channel kGimbalPitchChannel = Channel::LEFT_Y;
constexpr Channel kGimbalYawChannel = Channel::LEFT_X;
```

例如交换底盘前后和左右摇杆：

```cpp
constexpr Channel kChassisForwardChannel = Channel::RIGHT_X;
constexpr Channel kChassisLateralChannel = Channel::RIGHT_Y;
```

## 2. 通道反向

方向只使用 `1.0f` 或 `-1.0f`：

```cpp
constexpr float kChassisForwardChannelSign = -1.0f;
constexpr float kChassisLateralChannelSign = 1.0f;
constexpr float kChassisTurnChannelSign = -1.0f;
constexpr float kGimbalPitchChannelSign = -1.0f;
constexpr float kGimbalYawChannelSign = 1.0f;
```

前后反向：

```cpp
constexpr float kChassisForwardChannelSign = -1.0f;
```

## 3. 鼠标 X/Y/Z

底层库统一提供三个轴：

```cpp
remote::MouseAxis::X
remote::MouseAxis::Y
remote::MouseAxis::Z
```

默认云台映射：

```cpp
constexpr MouseAxis kGimbalPitchMouseAxis = MouseAxis::Y;
constexpr MouseAxis kGimbalYawMouseAxis = MouseAxis::X;
constexpr float kGimbalPitchMouseSign = -1.0f;
constexpr float kGimbalYawMouseSign = 1.0f;
```

例如用鼠标滚轮 Z 控制 Pitch：

```cpp
constexpr MouseAxis kGimbalPitchMouseAxis = MouseAxis::Z;
```

## 4. 键盘移动与功能键

```cpp
constexpr KeyboardKey kMoveForwardKey = KeyboardKey::W;
constexpr KeyboardKey kMoveBackwardKey = KeyboardKey::S;
constexpr KeyboardKey kMoveLeftKey = KeyboardKey::A;
constexpr KeyboardKey kMoveRightKey = KeyboardKey::D;
constexpr KeyboardKey kMoveSprintKey = KeyboardKey::SHIFT;
constexpr KeyboardKey kMoveCrouchKey = KeyboardKey::CTRL;
constexpr KeyboardKey kTurnLeftKey = KeyboardKey::Q;
constexpr KeyboardKey kTurnRightKey = KeyboardKey::E;

constexpr KeyboardKey kCycleModeKey = KeyboardKey::G;
constexpr KeyboardKey kManualModeKey = KeyboardKey::B;
constexpr KeyboardKey kSpinModeKey = KeyboardKey::V;
constexpr KeyboardKey kFollowModeKey = KeyboardKey::C;
constexpr KeyboardKey kFrictionToggleKey = KeyboardKey::Z;
constexpr KeyboardKey kUnloadKey = KeyboardKey::R;
constexpr KeyboardKey kUiClearKey = KeyboardKey::X;
constexpr KeyboardKey kFireKey = KeyboardKey::NONE;
```

`KeyboardKey::NONE` 用于关闭可选功能。

### 键盘速度曲线

键盘属于数字输入，按下后不会再直接输出 `0.5/1.0`。当前采用
“期望方向 + 加速 + 松键制动”的移动曲线：

```cpp
constexpr float kKeyboardCrouchCommand = 0.25f;
constexpr float kKeyboardCruiseCommand = 0.50f;
constexpr float kKeyboardSprintCommand = 0.85f;
constexpr float kKeyboardMoveAcceleration = 0.75f;
constexpr float kKeyboardMoveDeceleration = 1.50f;
constexpr float kKeyboardMoveBraking = 2.00f;
constexpr float kKeyboardMoveReverseRate = 3.50f;

constexpr float kKeyboardCrouchTurnCommand = 0.20f;
constexpr float kKeyboardCruiseTurnCommand = 0.35f;
constexpr float kKeyboardSprintTurnCommand = 0.45f;
constexpr float kKeyboardTurnAcceleration = 1.10f;
constexpr float kKeyboardTurnDeceleration = 1.80f;
constexpr float kKeyboardTurnBraking = 2.50f;
constexpr float kKeyboardTurnReverseRate = 3.50f;
```

速度档位：

```text
CTRL + WASD    慢速档 0.25
WASD           普通档 0.50
SHIFT + WASD   冲刺档 0.85
CTRL+SHIFT     慢速优先
```

三档切换均连续。升档使用线性加速率，降档使用独立减速率；松开移动键使用更强制动率，反向使用最快反向制动率。

默认 W 起步曲线约为：

```text
100 ms   0.075
250 ms   0.188
500 ms   0.375
667 ms   0.500
```

松开 W 后，从 `0.50` 回到零约需 `250 ms`。W+D 会先做二维归一化，
合成速度与单独按 W 相同。摇杆输入保持线性，并且会立即清除键盘残留速度。

调节原则：

```text
kKeyboardMoveAcceleration 调小：起步和升档更柔和
kKeyboardMoveDeceleration 调大：冲刺降档、CTRL 蹲下更快
kKeyboardMoveBraking 调大：松键停得更快
kKeyboardCrouchCommand 调小：CTRL 精细移动更慢
kKeyboardSprintCommand 调小：SHIFT 最高速度更低
kKeyboardCruiseTurnCommand 调小：普通 Q/E 转向更温和
```

## 5. 拨杆和鼠标按键

```cpp
constexpr SwitchBinding kPhysicalKillSwitch = {
    SwitchSide::RIGHT, remote::UP};
constexpr SwitchBinding kCycleModeSwitch = {
    SwitchSide::RIGHT, remote::DOWN};
constexpr SwitchBinding kFireSwitch = {
    SwitchSide::LEFT, remote::UP};
constexpr SwitchBinding kFrictionToggleSwitch = {
    SwitchSide::LEFT, remote::DOWN};

constexpr MouseButton kFireMouseButton = MouseButton::LEFT;
```

可选拨杆功能使用：

```cpp
constexpr SwitchBinding kCycleModeSwitch = DisabledSwitch();
```

物理 KILL 不能关闭。

## 6. 底层统一接口

`remote::RemoteInput` 提供：

```cpp
input.GetChannel(remote::Channel::RIGHT_X);
input.GetSwitch(remote::SwitchSide::LEFT);
input.KeyDown(remote::KeyboardKey::V);
input.GetMouseAxis(remote::MouseAxis::X);
input.GetMouseAxis(remote::MouseAxis::Y);
input.GetMouseAxis(remote::MouseAxis::Z);
input.MouseButtonDown(remote::MouseButton::LEFT);
```

原来的 `dbus->ch0`、`dbus->mouse.x`、`dbus->keyboard.bit.W` 等成员只为旧程序兼容保留，新代码统一使用 `GetInput()`。

## Chassis frame offset debugging

Atlas application commands use `forward/right`. The common chassis library uses
`SetSpeed(x_speed, y_speed, turn_speed)` with `x=right` and `y=forward`.
`chassis_task.cpp` now performs that fixed adaptation once at the sender boundary.

The additional physical installation offset is configured in:

```cpp
// programs/Atlas/gimbal/include/atlas_config.h
constexpr float kDefaultChassisFrameOffsetDeg = 0.0f;
```

Recommended test values:

```text
  0 deg  : configured front matches physical front
 +90 deg : rotate translation frame by +90 deg
 -90 deg : rotate translation frame by -90 deg
 180 deg : reverse forward and right
```

The runtime API can be used while chassis output is disabled:

```cpp
ChassisSetFrameOffsetDeg(90.0f);
ChassisSetFrameOffsetDeg(-90.0f);
const float current = ChassisGetFrameOffsetDeg();
```

`ChassisSetFrameOffsetDeg()` returns `false` while the chassis is enabled or when
the supplied value is not finite. Test with the robot lifted or in KILL.

Serial diagnostics now include:

```text
Chassis frame offset=0.0deg library_x(right)=... library_y(forward)=...
```

Do not use `control::chassis_t::offset` for this calibration. That field changes
the front/rear turn contribution and does not rotate the translation frame.
