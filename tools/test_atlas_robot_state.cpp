#include <cassert>
#include <cstdint>
#include <random>

#include "robot_state.h"

static RemoteControl_t Remote(bool kill = true) {
    RemoteControl_t remote;
    remote.online = true;
    remote.physical_kill = kill;
    remote.generation = 1;
    return remote;
}

static RefereeSnapshot_t Referee(uint16_t hp = 100,
                                  bool online = true,
                                  bool match_running = true) {
    RefereeSnapshot_t referee;
    referee.online = online;
    referee.match_running = match_running;
    referee.hp = hp;
    return referee;
}

static void ArmToManual(RemoteControl_t& remote,
                        RefereeSnapshot_t& referee,
                        uint32_t base_ms = 0) {
    remote.physical_kill = true;
    UpdateRobotMode(remote, referee, base_ms);
    assert(!robot_rearm_required);
    remote.physical_kill = false;
    UpdateRobotMode(remote, referee, base_ms + 10);
    assert(remote_mode == REMOTE_MODE_MANUAL);
}

int main() {
    InitRobotMode();
    auto remote = Remote(true);
    auto referee = Referee();
    ArmToManual(remote, referee);

    // Normal mode selection remains procedural and direct.
    remote.cycle_mode = true;
    UpdateRobotMode(remote, referee, 20);
    assert(remote_mode == REMOTE_MODE_SPIN);
    remote.cycle_mode = false;
    UpdateRobotMode(remote, referee, 30);

    // HP=0 has authority only while the referee is online and a match runs.
    referee.hp = 0;
    UpdateRobotMode(remote, referee, 40);
    assert(robot_hp_dead);
    assert(RobotIsKilled());
    assert(!robot_rearm_required);

    // Referee loss cancels HP authority immediately.  No valid HP>0 frame is
    // required, so this path cannot deadlock after death.
    referee.online = false;
    UpdateRobotMode(remote, referee, 50);
    assert(!robot_hp_dead);
    assert(!robot_rearm_required);
    assert(remote_mode == REMOTE_MODE_MANUAL);

    // Offline referee data, including hp=0, is ignored.
    referee.hp = 0;
    for (uint32_t time = 60; time < 300; time += 10) {
        UpdateRobotMode(remote, referee, time);
        assert(!robot_hp_dead);
        assert(remote_mode == REMOTE_MODE_MANUAL);
    }

    // Online but outside the active match: HP=0 is also ignored.
    referee.online = true;
    referee.match_running = false;
    referee.hp = 0;
    for (uint32_t time = 300; time < 500; time += 10) {
        UpdateRobotMode(remote, referee, time);
        assert(!robot_hp_dead);
        assert(remote_mode == REMOTE_MODE_MANUAL);
    }

    // If the match ends while death is latched, clear it immediately.
    referee.match_running = true;
    UpdateRobotMode(remote, referee, 500);
    assert(robot_hp_dead);
    assert(RobotIsKilled());
    referee.match_running = false;
    UpdateRobotMode(remote, referee, 510);
    assert(!robot_hp_dead);
    assert(remote_mode == REMOTE_MODE_MANUAL);

    // While a valid match continues, revival requires stable HP>0 for 300 ms.
    referee.match_running = true;
    referee.hp = 0;
    UpdateRobotMode(remote, referee, 600);
    assert(robot_hp_dead);
    referee.hp = 100;
    UpdateRobotMode(remote, referee, 610);
    UpdateRobotMode(remote, referee, 909);
    assert(robot_hp_dead);
    UpdateRobotMode(remote, referee, 910);
    assert(!robot_hp_dead);
    assert(remote_mode == REMOTE_MODE_MANUAL);
    assert(!robot_rearm_required);

    // Physical KILL always wins, including on referee recovery.
    referee.hp = 0;
    UpdateRobotMode(remote, referee, 1000);
    assert(robot_hp_dead);
    remote.physical_kill = true;
    referee.online = false;
    UpdateRobotMode(remote, referee, 1010);
    assert(!robot_hp_dead);
    assert(RobotIsKilled());
    remote.physical_kill = false;
    UpdateRobotMode(remote, referee, 1020);
    assert(remote_mode == REMOTE_MODE_MANUAL);

    // DBUS reconnect remains the only event requiring physical KILL re-arm.
    remote.generation = 2;
    remote.online = false;
    UpdateRobotMode(remote, referee, 1100);
    assert(RobotIsKilled());
    assert(robot_rearm_required);

    remote.online = true;
    remote.physical_kill = false;
    UpdateRobotMode(remote, referee, 1110);
    assert(RobotIsKilled());
    assert(robot_rearm_required);

    remote.physical_kill = true;
    UpdateRobotMode(remote, referee, 1120);
    assert(!robot_rearm_required);
    remote.physical_kill = false;
    UpdateRobotMode(remote, referee, 1130);
    assert(remote_mode == REMOTE_MODE_MANUAL);

    // Randomized safety test: referee can kill only in an active online match;
    // DBUS loss/physical KILL always kills and never depends on referee state.
    std::mt19937 random(42);
    for (uint32_t time = 1200; time < 200000; time += 10) {
        remote.cycle_mode = (random() % 97) == 0;
        remote.manual_mode = (random() % 173) == 0;
        remote.spin_mode = (random() % 181) == 0;
        remote.follow_mode = (random() % 191) == 0;

        if ((random() % 401) == 0) {
            remote.online = !remote.online;
            ++remote.generation;
        }
        if ((random() % 503) == 0) {
            remote.physical_kill = !remote.physical_kill;
        }
        if ((random() % 607) == 0) referee.online = !referee.online;
        if ((random() % 701) == 0) {
            referee.match_running = !referee.match_running;
        }
        if ((random() % 809) == 0) referee.hp = referee.hp == 0 ? 100 : 0;

        UpdateRobotMode(remote, referee, time);

        if (!referee.online || !referee.match_running) {
            assert(!robot_hp_dead);
        }
        if (!remote.online || remote.physical_kill || robot_rearm_required ||
            robot_hp_dead) {
            assert(RobotIsKilled());
        }
    }
    return 0;
}
