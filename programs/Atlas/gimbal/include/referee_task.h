#pragma once

#include <cstdint>

namespace communication {
class Referee;
class UserInterface;
}

/** Read-only subset of referee data used by Atlas control modules. */
struct RefereeSnapshot_t {
    bool online = false;
    bool match_running = false;
    bool chassis_power_enabled = true;
    bool gimbal_power_enabled = true;
    bool shooter_power_enabled = true;
    uint8_t robot_id = 0;
    uint16_t hp = 1;
    uint16_t max_hp = 0;
    uint16_t chassis_power_limit = 100;
    uint16_t chassis_power_buffer = 30;
    float chassis_power = 0.0f;
    uint32_t power_heat_sequence = 0U;
    uint16_t shooter_heat_limit = 200;
    uint16_t shooter_heat = 0;
    uint16_t bullets_17mm = 0;
    float bullet_speed = 0.0f;
};

void InitReferee();
uint8_t RefereeIsOnline();
RefereeSnapshot_t RefereeGetSnapshot();

/** Build the referee-backed UI transport without exposing the protocol object. */
communication::UserInterface* RefereeCreateUserInterface();
