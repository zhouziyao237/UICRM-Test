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

#pragma once

#include "main.h"

namespace control {

enum ShooterType_e {
    SHOOTER_TYPE_BURST = 0,
    SHOOTER_TYPE_COOL = 1,
};

enum HeatShootMode_e {
    HEAT_SHOOT_MODE_SINGLE = 0,
    HEAT_SHOOT_MODE_DOUBLE = 1,
    HEAT_SHOOT_MODE_RAPID = 2,
};

class HeatController {
public:
    struct Config_t {
        uint16_t heat_limit;
        uint16_t cool_rate;
        uint16_t buffer_heat;
        uint16_t heat_per_shot;
        float update_freq;
    };

    HeatController();
    explicit HeatController(ShooterType_e type);

    void SetShooterType(ShooterType_e type);
    void SetShootMode(HeatShootMode_e mode);
    void Update(uint16_t current_heat, uint16_t heat_limit);

    uint16_t GetMaxShootCount() const;
    bool CanShoot() const;
    bool CanShoot(uint16_t count) const;
    void RecordShot(uint16_t count = 1);

    uint16_t GetCurrentHeat() const;
    uint16_t GetSafeLimit() const;
    uint16_t GetHeatLimit() const;
    uint16_t GetBufferHeat() const;

private:
    Config_t config_;
    HeatShootMode_e shoot_mode_;
    uint16_t current_heat_;
    uint16_t predicted_heat_;
    uint32_t last_update_time_;
    uint16_t shots_recorded_;

    void InitBurstConfig();
    void InitCoolConfig();
    uint16_t GetHeatPerMode() const;
};

} /* namespace control */
