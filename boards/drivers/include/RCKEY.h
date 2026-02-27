// Copyright (c) 2026. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.
//
// Created by Administrator on 2026/3/2.
//

#pragma once

#include <cstdint>

// ==================== RC按键库头文件 ====================

/**
 * @brief RC按键触发模式枚举类型
 *
 * 定义了遥控器按键可能触发的各种事件模式，
 * 用于标识按键的不同操作状态和事件类型。
 */
typedef enum {
    RC_KEY_NONE = 0,        /**< 无触发事件 */
    RC_KEY_CLICK,           /**< 单击事件 */
    RC_KEY_DOUBLE_CLICK,    /**< 双击事件 */
    RC_KEY_LONG_PRESS,      /**< 长按事件 */
    RC_KEY_RELEASE          /**< 释放事件 */
} rc_key_mode_t;

// 按键状态
typedef enum {
    RC_KEY_STATE_IDLE = 0,
    RC_KEY_STATE_PRESSED,
    RC_KEY_STATE_RELEASE_WAIT
} rc_key_state_enum_t;

// 按键配置结构
typedef struct {
    uint8_t channel;           // 通道号
    uint16_t press_thres;      // 按下阈值
    uint16_t release_thres;    // 释放阈值
    uint32_t long_press_ms;    // 长按时间(ms)
    uint32_t double_click_ms;  // 双击超时(ms)
} rc_key_config_t;

// 按键状态机
typedef struct {
    rc_key_config_t config;
    rc_key_state_enum_t state;
    uint32_t press_time;
    uint32_t release_time;
    uint32_t last_process;
    bool is_pressed;
    rc_key_mode_t pending_event;
    uint32_t event_deadline;
    bool operation_done;
} rc_key_state_t;

// 回调函数类型
typedef void (*rc_key_callback_t)(uint8_t channel, rc_key_mode_t mode);

#define RC_MAX_KEYS 8
#define RC_PROCESS_INTERVAL 1
#define RC_MAX_CALLBACKS 8  // 新增：最大回调数量

// 新增：回调管理器结构
typedef struct {
    rc_key_callback_t callbacks[RC_MAX_CALLBACKS];
    uint8_t count;
} rc_callback_manager_t;

#ifdef __cplusplus
extern "C" {
#endif

    // ==================== 库接口函数 ====================

    // 初始化RC按键系统
    void rc_key_init(void);

    // 添加按键配置
    void rc_key_add(uint8_t channel, rc_key_config_t config);

    // 注册主回调函数（会清空之前的所有回调）
    void rc_key_register_callback(rc_key_callback_t callback);

    // 添加额外的回调函数（不会清空已有回调）
    void rc_key_add_callback(rc_key_callback_t callback);

    // 移除指定回调函数
    void rc_key_remove_callback(rc_key_callback_t callback);

    // 清空所有回调函数
    void rc_key_clear_callbacks(void);

    // 获取当前回调函数数量
    uint8_t rc_key_get_callback_count(void);

    // 处理待触发事件
    void rc_key_process_events(void);

    // 主要处理函数
    void rc_key_process(uint8_t channel, uint16_t value);

#ifdef __cplusplus
}
#endif
