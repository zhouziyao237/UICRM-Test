#include "referee_task.h"

#include "bsp_os.h"
#include "protocol.h"
#include "user_interface.h"
#include "sys.h"
#include "cmsis_os.h"
#include "task.h"

namespace {
communication::Referee* g_referee = nullptr;
}

void InitReferee() {
    bsp::UART* uart = atlas_referee_uart;
    RM_ASSERT_TRUE(uart != nullptr, "Referee init failed");
    g_referee = new communication::Referee(uart);
    osDelay(50);
}

uint8_t RefereeIsOnline() {
    const bool online = g_referee != nullptr && g_referee->IsOnline();
    return online ? 1U : 0U;
}

RefereeSnapshot_t RefereeGetSnapshot() {
    RefereeSnapshot_t snapshot;
    snapshot.online = RefereeIsOnline() != 0U;
    if (!snapshot.online || g_referee == nullptr) return snapshot;

    // Referee::ProcessDataRx publishes each protocol struct in a short
    // critical section. Hold the same scheduler/IRQ gate while composing the
    // Atlas subset so one snapshot never contains a partially copied struct.
    taskENTER_CRITICAL();
    snapshot.match_running = g_referee->game_status.game_progress == 4;
    snapshot.robot_id = g_referee->game_robot_status.robot_id;
    snapshot.hp = g_referee->game_robot_status.remain_HP;
    snapshot.max_hp = g_referee->game_robot_status.max_HP;
    snapshot.chassis_power_enabled =
        g_referee->game_robot_status.mains_power_chassis_output;
    snapshot.gimbal_power_enabled =
        g_referee->game_robot_status.mains_power_gimbal_output;
    snapshot.shooter_power_enabled =
        g_referee->game_robot_status.mains_power_shooter_output;
    snapshot.chassis_power_limit =
        g_referee->game_robot_status.chassis_power_limit;
    snapshot.chassis_power_buffer =
        g_referee->power_heat_data.chassis_power_buffer;
    snapshot.chassis_power = g_referee->power_heat_data.chassis_power;
    snapshot.power_heat_sequence = g_referee->GetPowerHeatSequence();
    snapshot.shooter_heat_limit =
        g_referee->game_robot_status.shooter_heat_limit;
    snapshot.shooter_heat =
        g_referee->power_heat_data.shooter_id1_17mm_cooling_heat;
    snapshot.bullets_17mm =
        g_referee->bullet_remaining.bullet_remaining_num_17mm;
    snapshot.bullet_speed = g_referee->shoot_data.bullet_speed;
    taskEXIT_CRITICAL();
    return snapshot;
}

communication::UserInterface* RefereeCreateUserInterface() {
    if (g_referee == nullptr || atlas_referee_uart == nullptr) {
        return nullptr;
    }
    return new communication::UserInterface(atlas_referee_uart,
                                            g_referee);
}
