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

#include <stdint.h>

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

/* ===== GAME_STATUS 0x0001 1Hz ===== */
typedef struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} __attribute__((packed)) game_status_t;

/* ===== GAME_RESULT 0x0002 ===== */
typedef struct {
    uint8_t winner;
} __attribute__((packed)) game_result_t;

/* ===== GAME_ROBOT_HP 0x0003 1Hz ===== */
typedef struct {
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} __attribute__((packed)) game_robot_HP_t;

/* ===== EVENT_DATA 0x0101 1Hz ===== */
typedef struct {
    uint32_t event_type;
} __attribute__((packed)) event_data_t;

/* ===== SUPPLY_PROJECTILE_ACTION 0x0102 ===== */
typedef struct {
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} __attribute__((packed)) supply_projectile_action_t;

/* ===== REFEREE_WARNING 0x0104 ===== */
typedef struct {
    uint8_t level;
    uint8_t offending_robot_id;
} __attribute__((packed)) referee_warning_t;

/* ===== DART_REMAINING_TIME 0x0105 1Hz ===== */
typedef struct {
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} __attribute__((packed)) dart_remaining_time_t;

/* ===== GAME_ROBOT_STATUS 0x0201 10Hz ===== */
typedef struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_cooling_rate;
    uint16_t shooter_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} __attribute__((packed)) game_robot_status_t;

/* ===== POWER_HEAT_DATA 0x0202 50Hz ===== */
typedef struct {
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} __attribute__((packed)) power_heat_data_t;

/* ===== GAME_ROBOT_POS 0x0203 10Hz ===== */
typedef struct {
    float x;
    float y;
    float yaw;
} __attribute__((packed)) game_robot_pos_t;

/* ===== BUFF 0x0204 1Hz ===== */
typedef struct {
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
} __attribute__((packed)) buff_t;

/* ===== AERIAL_ROBOT_ENERGY 0x0205 10Hz ===== */
typedef struct {
    uint8_t airforce_status;
    uint8_t time_remain;
} __attribute__((packed)) aerial_robot_energy_t;

/* ===== ROBOT_HURT 0x0206 ===== */
typedef struct {
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} __attribute__((packed)) robot_hurt_t;

/* ===== SHOOT_DATA 0x0207 ===== */
typedef struct {
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} __attribute__((packed)) shoot_data_t;

/* ===== BULLET_REMAINING 0x0208 10Hz ===== */
typedef struct {
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} __attribute__((packed)) bullet_remaining_t;

/* ===== RFID_STATUS 0x0209 1Hz ===== */
typedef struct {
    uint32_t rfid_status;
} __attribute__((packed)) rfid_status_t;

/* ===== DART_CLIENT_CMD 0x020A 10Hz ===== */
typedef struct {
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint16_t operate_launch_cmd_time;
} __attribute__((packed)) dart_client_cmd_t;

/* ===== GROUND_ROBOT_POSITION 0x020B 1Hz ===== */
typedef struct {
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
} __attribute__((packed)) ground_robot_position_t;

/* ===== RADAR_MARK 0x020C 1Hz ===== */
typedef struct {
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
} __attribute__((packed)) radar_mark_data_t;

/* ===== SENTRY_INFO 0x020D 1Hz ===== */
typedef struct {
    uint32_t sentry_info;
} __attribute__((packed)) sentry_info_t;

/* ===== RADAR_INFO 0x020E 1Hz ===== */
typedef struct {
    uint32_t radar_info;
} __attribute__((packed)) radar_info_t;

/* ===== INTERACTIVE_DATA 0x0301 ===== */
typedef struct {
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} __attribute__((packed)) robot_interactive_data_header_t;

/* ===== Start Sub Command ===== */

/* ===== ROBOT_COMMUNICATION 0x0200-0x02FF ===== */
typedef struct {
    robot_interactive_data_header_t header;
    uint8_t data[];
} __attribute__((packed)) robot_interactive_data_t;

/* ===== GRAPHIC_DELETE 0x0100 ===== */
typedef struct {
    robot_interactive_data_header_t header;
    uint8_t operate_type;
    uint8_t layer;
} __attribute__((packed)) graphic_delete_t;

/* ===== GRAPHIC_DATA ===== */
typedef struct {
    uint8_t graphic_name[3];
    uint32_t operate_type : 3;
    uint32_t graphic_type : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11;
} __attribute__((packed)) graphic_data_t;

/* ===== GRAPHIC_SINGLE 0x0101 ===== */
typedef struct {
    robot_interactive_data_header_t header;
    graphic_data_t graphic_data_struct;
} __attribute__((packed)) graphic_single_t;

/* ===== GRAPHIC_DOUBLE 0x0102 ===== */
typedef struct {
    robot_interactive_data_header_t header;
    graphic_data_t graphic_data_struct[2];
} __attribute__((packed)) graphic_double_t;

/* ===== GRAPHIC_FIVE 0x0105 ===== */
typedef struct {
    robot_interactive_data_header_t header;
    graphic_data_t graphic_data_struct[5];
} __attribute__((packed)) graphic_five_t;

/* ===== GRAPHIC_SEVEN 0x0107 ===== */
typedef struct {
    robot_interactive_data_header_t header;
    graphic_data_t graphic_data_struct[7];
} __attribute__((packed)) graphic_seven_t;

/* ===== GRAPHIC_CHARACTER 0x0110 ===== */
typedef struct {
    robot_interactive_data_header_t header;
    graphic_data_t graphic_data_struct;
    uint8_t data[30];
} __attribute__((packed)) graphic_character_t;

enum content {
    NO_GRAPH,
    DELETE_GRAPH,
    SINGLE_GRAPH,
    DOUBLE_GRAPH,
    FIVE_GRAPH,
    SEVEN_GRAPH,
    CHAR_GRAPH,
};

/* ===== SENTRY_DECISION 0x0120 ===== */
typedef struct {
    uint32_t sentry_cmd;
} __attribute__((packed)) sentry_cmd_t;

/* ===== RADAR_DECISION 0x0121 ===== */
typedef struct {
    uint32_t radar_cmd;
} __attribute__((packed)) radar_cmd_t;

/* ===== End Sub Command ===== */

/* ===== CUSTOM_ROBOT_DATA 0x0302 ===== */
typedef struct {
    uint8_t data[30];
} __attribute__((packed)) custom_robot_data_t;

/* ===== MAP_COMMAND 0x0303 ===== */
typedef struct {
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t cmd_source;
} __attribute__((packed)) map_command_t;

/* ===== REMOTE_CONTROL_DATA 0x0304 30Hz ===== */
// 注意：这个结构体依赖于外部定义的mouse_t和keyboard_t类型
typedef struct {
    // remote::mouse_t mouse;
    // remote::keyboard_t keyboard;
    uint8_t mouse_data[12];     // 鼠标数据占位符
    uint8_t keyboard_data[10];  // 键盘数据占位符
    uint16_t reserved;
} __attribute__((packed)) remote_control_t;

/* ===== MAP_ROBOT_DATA 0x0305 ===== */
typedef struct {
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
} __attribute__((packed)) map_robot_data_t;

/* ===== CUSTOM_CLIENT_DATA 0x0306 ===== */
typedef struct {
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved;
} __attribute__((packed)) custom_client_data_t;

/* ===== CUSTOM_CLIENT_MAP_COMMAND 0x0307 ===== */
typedef struct {
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
} __attribute__((packed)) custom_client_map_command_t;

/* ===== CUSTOM_INFO 0x0308 ===== */
typedef struct {
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} __attribute__((packed)) custom_info_t;

class Referee {
  public:
    game_status_t game_status{};                            // 0x0001
    game_result_t game_result{};                            // 0x0002
    game_robot_HP_t game_robot_HP{};                        // 0x0003
    event_data_t event_data{};                              // 0x0101
    supply_projectile_action_t supply_projectile_action{};  // 0x0102
    referee_warning_t referee_warning{};                    // 0x0104
    dart_remaining_time_t dart_remaining_time{};            // 0x0105
    game_robot_status_t game_robot_status{};                // 0x0201
    power_heat_data_t power_heat_data{};                    // 0x0202
    game_robot_pos_t game_robot_pos{};                      // 0x0203
    buff_t buff{};                                          // 0x0204
    aerial_robot_energy_t aerial_robot_energy{};            // 0x0205
    robot_hurt_t robot_hurt{};                              // 0x0206
    shoot_data_t shoot_data{};                              // 0x0207
    bullet_remaining_t bullet_remaining{};                  // 0x0208
    rfid_status_t rfid_status{};                            // 0x0209
    dart_client_cmd_t dart_client_cmd{};                    // 0x020A
    ground_robot_position_t ground_robot_position{};        // 0x020B
    radar_mark_data_t radar_mark_data{};                    // 0x020C
    sentry_info_t sentry_info{};                            // 0x020D
    radar_info_t radar_info{};                              // 0x020E

    graphic_delete_t graphic_delete{};
    graphic_single_t graphic_single{};
    graphic_double_t graphic_double{};
    graphic_five_t graphic_five{};
    graphic_seven_t graphic_seven{};
    graphic_character_t graphic_character{};

    custom_robot_data_t custom_robot_data{};                  // 0x0302
    map_command_t map_command{};                              // 0x0303
    remote_control_t remote_control{};                        // 0x0304
    map_robot_data_t map_robot_data{};                        // 0x0305
    custom_client_data_t custom_client_data{};                // 0x0306
    custom_client_map_command_t custom_client_map_command{};  // 0x0307
    custom_info_t custom_info{};
};