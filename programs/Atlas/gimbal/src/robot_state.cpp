#include "robot_state.h"

#include "utils.h"

namespace {
constexpr uint32_t kHpRecoveryDebounceMs = 300;

bool generation_valid = false;
uint32_t observed_generation = 0;
bool hp_recovery_pending = false;
uint32_t hp_recovery_start_ms = 0;

BoolEdgeDetector cycle_edge(false);
BoolEdgeDetector manual_edge(false);
BoolEdgeDetector spin_edge(false);
BoolEdgeDetector follow_edge(false);

void SyncModeEdges(const RemoteControl_t& remote) {
    cycle_edge.reset(remote.cycle_mode);
    manual_edge.reset(remote.manual_mode);
    spin_edge.reset(remote.spin_mode);
    follow_edge.reset(remote.follow_mode);
}

void SaveLastRunMode() {
    if (remote_mode != REMOTE_MODE_KILL) {
        last_remote_mode = remote_mode;
    }
}

void EnterKill() {
    SaveLastRunMode();
    remote_mode = REMOTE_MODE_KILL;
}

bool UpdateHpState(const RefereeSnapshot_t& referee, uint32_t now_ms) {
    const bool hp_gate_active = referee.online && referee.match_running;

    // Referee data is allowed to stop the robot only while a valid match is
    // actively running.  Link loss, pre-match, timeout and settlement are all
    // treated as "no HP authority" and must never leave a stale death latch.
    if (!hp_gate_active) {
        const bool was_dead = robot_hp_dead;
        robot_hp_dead = false;
        hp_recovery_pending = false;
        hp_recovery_start_ms = 0;
        return was_dead;
    }

    if (referee.hp == 0) {
        robot_hp_dead = true;
        hp_recovery_pending = false;
        hp_recovery_start_ms = 0;
        return false;
    }

    if (!robot_hp_dead) {
        hp_recovery_pending = false;
        hp_recovery_start_ms = 0;
        return false;
    }

    // During an active match, require a short stable HP>0 window so one
    // partially updated referee frame cannot re-enable the robot.
    if (!hp_recovery_pending) {
        hp_recovery_pending = true;
        hp_recovery_start_ms = now_ms;
        return false;
    }

    if (now_ms - hp_recovery_start_ms < kHpRecoveryDebounceMs) {
        return false;
    }

    robot_hp_dead = false;
    hp_recovery_pending = false;
    hp_recovery_start_ms = 0;
    return true;
}
}  // namespace

volatile RemoteMode_e remote_mode = REMOTE_MODE_KILL;
volatile RemoteMode_e last_remote_mode = REMOTE_MODE_MANUAL;
volatile bool robot_hp_dead = false;
volatile bool robot_rearm_required = true;

void InitRobotMode() {
    remote_mode = REMOTE_MODE_KILL;
    last_remote_mode = REMOTE_MODE_MANUAL;
    robot_hp_dead = false;
    robot_rearm_required = true;

    generation_valid = false;
    observed_generation = 0;
    hp_recovery_pending = false;
    hp_recovery_start_ms = 0;

    cycle_edge.reset(false);
    manual_edge.reset(false);
    spin_edge.reset(false);
    follow_edge.reset(false);
}

RemoteMode_e NextRemoteMode(RemoteMode_e mode) {
    switch (mode) {
        case REMOTE_MODE_MANUAL:
            return REMOTE_MODE_SPIN;
        case REMOTE_MODE_SPIN:
            return REMOTE_MODE_FOLLOW;
        case REMOTE_MODE_FOLLOW:
        default:
            return REMOTE_MODE_MANUAL;
    }
}

void SetRobotMode(RemoteMode_e mode) {
    if (mode == REMOTE_MODE_KILL) {
        EnterKill();
        return;
    }
    remote_mode = mode;
    last_remote_mode = mode;
}

void UpdateRobotMode(const RemoteControl_t& remote,
                     const RefereeSnapshot_t& referee,
                     uint32_t now_ms) {
    const bool hp_recovered = UpdateHpState(referee, now_ms);
    if (hp_recovered) {
        // Preserve MANUAL as the post-revival mode even if the operator is
        // currently holding physical KILL or the DBUS link is being rearmed.
        last_remote_mode = REMOTE_MODE_MANUAL;
    }

    if (!generation_valid) {
        generation_valid = true;
        observed_generation = remote.generation;
        robot_rearm_required = true;
    } else if (observed_generation != remote.generation) {
        observed_generation = remote.generation;
        robot_rearm_required = true;
        // Any DBUS disconnect/reconnect always restarts from MANUAL.
        last_remote_mode = REMOTE_MODE_MANUAL;
        EnterKill();
    }

    if (robot_rearm_required) {
        EnterKill();
        SyncModeEdges(remote);
        if (remote.online && remote.physical_kill) {
            // Boot/reconnect acknowledgement: pass through the physical KILL position.
            // Leaving KILL after this acknowledgement always starts in MANUAL.
            robot_rearm_required = false;
            last_remote_mode = REMOTE_MODE_MANUAL;
        }
        return;
    }

    if (!remote.online || remote.physical_kill || robot_hp_dead) {
        EnterKill();
        SyncModeEdges(remote);
        return;
    }

    if (hp_recovered) {
        // Referee-confirmed revival does not require a physical KILL cycle.
        // It always returns to MANUAL instead of restoring SPIN/FOLLOW.
        remote_mode = REMOTE_MODE_MANUAL;
        last_remote_mode = REMOTE_MODE_MANUAL;
        SyncModeEdges(remote);
        return;
    }

    if (remote_mode == REMOTE_MODE_KILL) {
        remote_mode = last_remote_mode == REMOTE_MODE_KILL
                          ? REMOTE_MODE_MANUAL
                          : last_remote_mode;
    }

    cycle_edge.input(remote.cycle_mode);
    manual_edge.input(remote.manual_mode);
    spin_edge.input(remote.spin_mode);
    follow_edge.input(remote.follow_mode);

    if (cycle_edge.posEdge()) remote_mode = NextRemoteMode(remote_mode);
    if (follow_edge.posEdge()) remote_mode = REMOTE_MODE_FOLLOW;
    if (spin_edge.posEdge()) remote_mode = REMOTE_MODE_SPIN;
    if (manual_edge.posEdge()) remote_mode = REMOTE_MODE_MANUAL;

    SaveLastRunMode();
}

const char* RobotModeName(RemoteMode_e mode) {
    switch (mode) {
        case REMOTE_MODE_KILL:
            return "KILL";
        case REMOTE_MODE_MANUAL:
            return "MANUAL";
        case REMOTE_MODE_SPIN:
            return "SPIN";
        case REMOTE_MODE_FOLLOW:
            return "FOLLOW";
        case REMOTE_MODE_AUTOAIM:
            return "AUTOAIM";
        case REMOTE_MODE_AUTO:
            return "AUTO";
        default:
            return "UNKNOWN";
    }
}
