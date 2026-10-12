#include "sys_task.h"

#include "gimbal_data.h"
#include "cmsis_os.h"
#include "task.h"

namespace cfg = xuanshu::gimbal_config;

namespace {
AppStateSnapshot_t frames[2]{};
volatile uint8_t active_frame = 0U;
AppStateSnapshot_t working{};
bool rc_generation_valid = false;

void Publish() {
    taskENTER_CRITICAL();
    const uint8_t next = static_cast<uint8_t>(active_frame ^ 1U);
    ++working.generation;
    frames[next] = working;
    active_frame = next;
    taskEXIT_CRITICAL();
}
}  // namespace

osThreadId_t SysTaskHandle;

void InitAppState() {
    working = {};
    working.mode = GimbalMode::KILL;
    working.rearm_required = true;
    frames[0] = working;
    frames[1] = working;
}

void SysTask(void* arg) {
    UNUSED(arg);

    while (true) {
        const RcSnapshot_t rc = RCGetSnapshot();

        if (!rc_generation_valid) {
            rc_generation_valid = true;
            working.rc_generation = rc.generation;
            working.rearm_required = true;
        } else if (working.rc_generation != rc.generation) {
            working.rc_generation = rc.generation;
            working.rearm_required = true;
            working.mode = GimbalMode::KILL;
        }

        if (!rc.online) {
            working.mode = GimbalMode::KILL;
            working.rearm_required = true;
            Publish();
            osDelay(cfg::kSysPeriodMs);
            continue;
        }

        if (working.rearm_required) {
            working.mode = GimbalMode::KILL;
            // Boot/reconnect acknowledgement: the operator must deliberately
            // pass through physical KILL before the motors can be enabled.
            if (rc.physical_kill) working.rearm_required = false;
            Publish();
            osDelay(cfg::kSysPeriodMs);
            continue;
        }

        working.mode = rc.physical_kill ? GimbalMode::KILL : rc.requested_mode;
        Publish();
        osDelay(cfg::kSysPeriodMs);
    }
}

AppStateSnapshot_t AppStateGetSnapshot() {
    taskENTER_CRITICAL();
    const AppStateSnapshot_t snapshot = frames[active_frame];
    taskEXIT_CRITICAL();
    return snapshot;
}

bool AppIsKilled() {
    return AppStateGetSnapshot().mode == GimbalMode::KILL;
}
