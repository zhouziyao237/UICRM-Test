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

/* Protocol numbers used both by the wire format, config system, and
   field setters.
*/

/* Command for Referee */

/*
 * 0x0001 GAME_STATUS
 * 0x0002 GAME_RESULT
 * 0x0003 GAME_ROBOT_HP
 * 0x0005 ICRA_BUFF_DEBUFF_ZONE_STATUS [X]
 * 0x0101 EVENT_DATA
 * 0x0102 SUPPLY_PROJECTILE_ACTION
 * 0x0104 REFEREE_WARNING
 * 0x0105 DART_REMAINING_TIME
 * 0x0201 GAME_ROBOT_STATUS
 * 0x0202 POWER_HEAT_DATA
 * 0x0203 GAME_ROBOT_POS
 * 0x0204 BUFF
 * 0x0205 AERIAL_ROBOT_ENERGY
 * 0x0206 ROBOT_HURT
 * 0x0207 SHOOT_DATA
 * 0x0208 BULLET_REMAINING
 * 0x0209 RFID_STATUS
 * 0x020A DART_CLIENT_CMD
 * 0x0301 STUDENT_INTERACTIVE
 * 0x0302 ROBOT_INTERACTIVE [x]
 * 0x0303 ROBOT_COMMAND [x]
 * 0x0304 REMOTE_CONTROL_DATA
 * 0x0305 CLIENT_MAP_COMMAND [x]
 */

/* TODO(neo): above information with [x] should be implemented when needed in the future*/

#define GAME_STATUS 0x0001
#define GAME_RESULT 0x0002
#define GAME_ROBOT_HP 0x0003
#define EVENT_DATA 0x0101
#define SUPPLY_PROJECTILE_ACTION 0x0102
#define REFEREE_WARNING 0x0104
#define DART_REMAINING_TIME 0x0105
#define GAME_ROBOT_STATUS 0x0201
#define POWER_HEAT_DATA 0x0202
#define GAME_ROBOT_POS 0x0203
#define BUFF 0x0204
#define AERIAL_ROBOT_ENERGY 0x0205
#define ROBOT_HURT 0x0206
#define SHOOT_DATA 0x0207
#define BULLET_REMAINING 0x0208
#define RFID_STATUS 0x0209
#define DART_CLIENT_CMD 0x020A
#define STUDENT_INTERACTIVE 0x0301
#define REMOTE_CONTROL_DATA 0x0304

/* Sub Command for Interactive Data (0x0301) */

/* ===== ROBOT_COMMUNICATION 0x0200-0x02FF ===== */
#define ROBOT_INTERACTIVE_DATA 0x0200  // Base for robot interactive data

/* ===== GRAPHIC_DELETE 0x0100 ===== */
#define GRAPHIC_DELETE 0x0100

/* ===== GRAPHIC_DATA ===== */
// Graphic operation types
#define GRAPHIC_ADD 1
#define GRAPHIC_UPDATE 2
#define GRAPHIC_DELETE_SINGLE 3
#define GRAPHIC_DELETE_ALL 4

// Graphic types
#define GRAPHIC_LINE 0
#define GRAPHIC_RECTANGLE 1
#define GRAPHIC_CIRCLE 2
#define GRAPHIC_ELLIPSE 3
#define GRAPHIC_ARC 4
#define GRAPHIC_FLOAT 5
#define GRAPHIC_INTEGER 6
#define GRAPHIC_CHARACTER 7

/* ===== GRAPHIC_SINGLE 0x0101 ===== */
#define GRAPHIC_SINGLE 0x0101

/* ===== GRAPHIC_DOUBLE 0x0102 ===== */
#define GRAPHIC_DOUBLE 0x0102

/* ===== GRAPHIC_FIVE 0x0105 ===== */
#define GRAPHIC_FIVE 0x0105

/* ===== GRAPHIC_SEVEN 0x0107 ===== */
#define GRAPHIC_SEVEN 0x0107

/* ===== GRAPHIC_CHARACTER 0x0110 ===== */
#define GRAPHIC_CHARACTER_CMD 0x0110

/* ===== SENTRY_DECISION 0x0120 ===== */
#define SENTRY_CMD 0x0120

/* ===== RADAR_DECISION 0x0121 ===== */
#define RADAR_CMD 0x0121

/* ===== CUSTOM_ROBOT_DATA 0x0302 ===== */
#define CUSTOM_ROBOT_DATA 0x0302

/* ===== MAP_COMMAND 0x0303 ===== */
#define MAP_COMMAND 0x0303

/* ===== REMOTE_CONTROL_DATA 0x0304 ===== */
#define REMOTE_CONTROL_DATA_CMD 0x0304

/* ===== MAP_ROBOT_DATA 0x0305 ===== */
#define MAP_ROBOT_DATA 0x0305

/* ===== CUSTOM_CLIENT_DATA 0x0306 ===== */
#define CUSTOM_CLIENT_DATA 0x0306

/* ===== CUSTOM_CLIENT_MAP_COMMAND 0x0307 ===== */
#define CUSTOM_CLIENT_MAP_COMMAND 0x0307

/* ===== CUSTOM_INFO 0x0308 ===== */
#define CUSTOM_INFO 0x0308
