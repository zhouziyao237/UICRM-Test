#pragma once

#include <cstdint>

#include "rc.h"
#include "referee_task.h"

/**
 * Atlas has one run-mode variable only.
 * DBUS loss requires a physical KILL acknowledgement and then starts MANUAL.
 * Referee HP=0 can force KILL only while the referee link is online and
 * game_progress reports an active match.  Leaving the match or losing the
 * referee link clears the HP latch and returns to MANUAL without re-arming.
 *
 * Add a new mode in three obvious places:
 * 1. Add an enum value here.
 * 2. Add it to NextRemoteMode() in robot_state.cpp if it participates in cycling.
 * 3. Add a case in chassis_task.cpp and/or gimbal_task.cpp.
 */
enum RemoteMode_e : uint8_t {
    REMOTE_MODE_KILL = 0,
    REMOTE_MODE_MANUAL,
    REMOTE_MODE_SPIN,
    REMOTE_MODE_FOLLOW,
    REMOTE_MODE_AUTOAIM,
    REMOTE_MODE_AUTO,
};

struct RobotStateSnapshot_t {
    RemoteMode_e mode = REMOTE_MODE_KILL;
    RemoteMode_e last_mode = REMOTE_MODE_MANUAL;
    bool hp_dead = false;
    bool rearm_required = true;
    uint32_t generation = 0U;
};

extern volatile RemoteMode_e remote_mode;
extern volatile RemoteMode_e last_remote_mode;
extern volatile bool robot_hp_dead;
extern volatile bool robot_rearm_required;

void InitRobotMode();
void UpdateRobotMode(const RemoteControl_t& remote,
                     const RefereeSnapshot_t& referee,
                     uint32_t now_ms);
void SetRobotMode(RemoteMode_e mode);
RemoteMode_e NextRemoteMode(RemoteMode_e mode);
RobotStateSnapshot_t RobotGetSnapshot();

inline bool RobotIsKilled() {
    return RobotGetSnapshot().mode == REMOTE_MODE_KILL;
}

inline bool RobotIsHpDead() {
    return RobotGetSnapshot().hp_dead;
}

inline bool RobotCanRun() {
    const RobotStateSnapshot_t state = RobotGetSnapshot();
    return state.mode != REMOTE_MODE_KILL && !state.hp_dead;
}

const char* RobotModeName(RemoteMode_e mode);
