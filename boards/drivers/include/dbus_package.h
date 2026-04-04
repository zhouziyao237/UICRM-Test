/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # DBUS wire-level payload types.                           #
 ###########################################################*/

#pragma once

#include "main.h"
#include "remote_input.h"

namespace remote {

// DBUS wire mouse payload. DBUS has left/right buttons and no middle button.
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t l;
    uint8_t r;
} __packed mouse_t;

#define mouse_xy_max 32767.0

}  // namespace remote
