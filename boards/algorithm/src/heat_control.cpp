/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "heat_control.h"
#include "bsp_os.h"

namespace control {

HeatController::HeatController()
    : shoot_mode_(HEAT_SHOOT_MODE_SINGLE),
      current_heat_(0),
      predicted_heat_(0),
      last_update_time_(0),
      shots_recorded_(0) {
    InitBurstConfig();
}

HeatController::HeatController(ShooterType_e type)
    : shoot_mode_(HEAT_SHOOT_MODE_SINGLE),
      current_heat_(0),
      predicted_heat_(0),
      last_update_time_(0),
      shots_recorded_(0) {
    SetShooterType(type);
}

void HeatController::InitBurstConfig() {
    config_.heat_limit = 230;
    config_.cool_rate = 14;
    config_.buffer_heat = 30;
    config_.heat_per_shot = 10;
    config_.update_freq = 10.0f;
}

void HeatController::InitCoolConfig() {
    config_.heat_limit = 88;
    config_.cool_rate = 24;
    config_.buffer_heat = 20;
    config_.heat_per_shot = 10;
    config_.update_freq = 10.0f;
}

void HeatController::SetShooterType(ShooterType_e type) {
    if (type == SHOOTER_TYPE_BURST) {
        InitBurstConfig();
    } else {
        InitCoolConfig();
    }
}

void HeatController::SetShootMode(HeatShootMode_e mode) {
    shoot_mode_ = mode;
}

void HeatController::Update(uint16_t current_heat, uint16_t heat_limit) {
    current_heat_ = current_heat;
    if (heat_limit > 0) {
        config_.heat_limit = heat_limit;
    }
    predicted_heat_ = current_heat_;
    last_update_time_ = bsp::GetHighresTickMilliSec();
    shots_recorded_ = 0;
}

uint16_t HeatController::GetHeatPerMode() const {
    switch (shoot_mode_) {
        case HEAT_SHOOT_MODE_DOUBLE:
            return config_.heat_per_shot * 2;
        case HEAT_SHOOT_MODE_RAPID:
        case HEAT_SHOOT_MODE_SINGLE:
        default:
            return config_.heat_per_shot;
    }
}

uint16_t HeatController::GetMaxShootCount() const {
    uint16_t safe_limit = GetSafeLimit();
    if (predicted_heat_ >= safe_limit) {
        return 0;
    }
    uint16_t heat_per_mode = GetHeatPerMode();
    if (heat_per_mode == 0) {
        return 0;
    }
    uint16_t available_heat = safe_limit - predicted_heat_;
    uint16_t max_shots = available_heat / heat_per_mode;
    if (shoot_mode_ == HEAT_SHOOT_MODE_DOUBLE) {
        return max_shots * 2;
    }
    return max_shots;
}

bool HeatController::CanShoot() const {
    return CanShoot(1);
}

bool HeatController::CanShoot(uint16_t count) const {
    uint16_t safe_limit = GetSafeLimit();
    uint16_t heat_cost = count * config_.heat_per_shot;
    return (predicted_heat_ + heat_cost) <= safe_limit;
}

void HeatController::RecordShot(uint16_t count) {
    shots_recorded_ += count;
    predicted_heat_ += count * config_.heat_per_shot;
    if (predicted_heat_ > config_.heat_limit) {
        predicted_heat_ = config_.heat_limit;
    }
}

uint16_t HeatController::GetCurrentHeat() const {
    return current_heat_;
}

uint16_t HeatController::GetSafeLimit() const {
    if (config_.heat_limit > config_.buffer_heat) {
        return config_.heat_limit - config_.buffer_heat;
    }
    return config_.heat_limit;
}

uint16_t HeatController::GetHeatLimit() const {
    return config_.heat_limit;
}

uint16_t HeatController::GetBufferHeat() const {
    return config_.buffer_heat;
}

} /* namespace control */
