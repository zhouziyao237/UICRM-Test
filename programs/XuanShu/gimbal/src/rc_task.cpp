#include "rc_task.h"

#include <cmath>

#include "dbus.h"
#include "gimbal_data.h"
#include "bsp_error_handler.h"
#include "main.h"
#include "cmsis_os.h"
#include "task.h"

namespace cfg = xuanshu::gimbal_config;

namespace {
remote::DBUS* dbus = nullptr;
RcSnapshot_t frames[2]{};
volatile uint8_t active_frame = 0U;
RcSnapshot_t working{};
bool was_online = false;

float Normalize(int16_t value, int16_t full_scale, float deadband) {
    if (full_scale <= 0) return 0.0f;
    float normalized = static_cast<float>(value) / static_cast<float>(full_scale);
    if (std::fabs(normalized) <= deadband) return 0.0f;
    normalized = normalized > 0.0f
                     ? (normalized - deadband) / (1.0f - deadband)
                     : (normalized + deadband) / (1.0f - deadband);
    if (normalized > 1.0f) normalized = 1.0f;
    if (normalized < -1.0f) normalized = -1.0f;
    return normalized;
}

void Publish() {
    taskENTER_CRITICAL();
    const uint8_t next = static_cast<uint8_t>(active_frame ^ 1U);
    frames[next] = working;
    active_frame = next;
    taskEXIT_CRITICAL();
}

GimbalMode DecodeMode(const remote::RemoteInput& input) {
    if (!input.HasSwitch(cfg::kModeSwitch)) return GimbalMode::KILL;
    const remote::switch_t position = input.GetSwitch(cfg::kModeSwitch);
    if (position == cfg::kKillPosition) return GimbalMode::KILL;
    if (position == cfg::kAutoAimPosition) return GimbalMode::AUTOAIM;
    return GimbalMode::MANUAL;
}

void MapManualInput(const remote::RemoteInput& input) {
    const float mouse_pitch = Normalize(
        input.GetMouseAxis(remote::MouseAxis::Y), remote::RemoteInput::MOUSE_AXIS_MAX,
        cfg::kMouseDeadband);
    const float mouse_yaw = Normalize(
        input.GetMouseAxis(remote::MouseAxis::X), remote::RemoteInput::MOUSE_AXIS_MAX,
        cfg::kMouseDeadband);

    if (mouse_pitch != 0.0f || mouse_yaw != 0.0f) {
        working.pitch_rate_rad_s =
            cfg::kMousePitchSign * mouse_pitch * cfg::kPitchMouseRateRadS;
        working.yaw_rate_rad_s =
            cfg::kMouseYawSign * mouse_yaw * cfg::kYawMouseRateRadS;
        return;
    }

    const float pitch = Normalize(input.GetChannel(cfg::kPitchChannel),
                                  remote::RemoteInput::CHANNEL_MAX,
                                  cfg::kStickDeadband);
    const float yaw = Normalize(input.GetChannel(cfg::kYawChannel),
                                remote::RemoteInput::CHANNEL_MAX,
                                cfg::kStickDeadband);
    working.pitch_rate_rad_s =
        cfg::kPitchChannelSign * pitch * cfg::kPitchStickRateRadS;
    working.yaw_rate_rad_s =
        cfg::kYawChannelSign * yaw * cfg::kYawStickRateRadS;
}

void ResetForOffline(bool generation_change) {
    if (generation_change) ++working.generation;
    working.online = false;
    working.physical_kill = true;
    working.requested_mode = GimbalMode::KILL;
    working.pitch_rate_rad_s = 0.0f;
    working.yaw_rate_rad_s = 0.0f;
}
}  // namespace

osThreadId_t RCTaskHandle;

void InitRC() {
    dbus = new remote::DBUS(&huart3);
    RM_ASSERT_TRUE(dbus != nullptr, "XuanShu DBUS init failed");
    dbus->SetThreshold(cfg::kDbusOfflineTimeoutMs);
    ResetForOffline(false);
    frames[0] = working;
    frames[1] = working;
}

void RCTask(void* arg) {
    UNUSED(arg);
    while (true) {
        const bool online = dbus != nullptr && dbus->IsOnline();
        if (online != was_online) {
            was_online = online;
            ResetForOffline(true);
        }

        if (online) {
            const remote::RemoteInput input = dbus->GetInput();
            working.online = true;
            working.requested_mode = DecodeMode(input);
            working.physical_kill =
                working.requested_mode == GimbalMode::KILL;
            MapManualInput(input);
        } else {
            ResetForOffline(false);
        }

        Publish();
        osDelay(cfg::kRcPeriodMs);
    }
}

RcSnapshot_t RCGetSnapshot() {
    taskENTER_CRITICAL();
    const RcSnapshot_t snapshot = frames[active_frame];
    taskEXIT_CRITICAL();
    return snapshot;
}

const char* GimbalModeName(GimbalMode mode) {
    switch (mode) {
        case GimbalMode::KILL:
            return "KILL";
        case GimbalMode::MANUAL:
            return "MANUAL";
        case GimbalMode::AUTOAIM:
            return "AUTOAIM";
        default:
            return "UNKNOWN";
    }
}
