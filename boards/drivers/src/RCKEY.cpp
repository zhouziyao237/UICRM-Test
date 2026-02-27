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

#include "RCKEY.h"

#include "bsp_os.h"

// ==================== RC按键库实现 ====================

// 全局变量
static rc_key_state_t rc_keys[RC_MAX_KEYS];
static uint8_t rc_key_count = 0;
static rc_callback_manager_t callback_manager = {};

// 默认配置
static const rc_key_config_t rc_default_config = {
    .channel = 0,
    .press_thres = 1750,
    .release_thres = 1250,
    .long_press_ms = 1000,
    .double_click_ms = 500
};

// ==================== 库接口函数实现 ====================

void rc_key_init(void) {
    rc_key_count = 0;
    callback_manager.count = 0;  // 初始化回调管理器

    for (int i = 0; i < RC_MAX_KEYS; i++) {
        rc_keys[i].config = rc_default_config;
        rc_keys[i].state = RC_KEY_STATE_IDLE;
        rc_keys[i].press_time = 0;
        rc_keys[i].release_time = 0;
        rc_keys[i].last_process = 0;
        rc_keys[i].is_pressed = false;
        rc_keys[i].pending_event = RC_KEY_NONE;
        rc_keys[i].event_deadline = 0;
        rc_keys[i].operation_done = false;
    }
}

void rc_key_add(uint8_t channel, rc_key_config_t config) {
    if (rc_key_count < RC_MAX_KEYS) {
        rc_keys[rc_key_count].config = config;
        rc_keys[rc_key_count].config.channel = channel;
        rc_key_count++;
    }
}

void rc_key_register_callback(rc_key_callback_t callback) {
    // 清空所有现有回调，注册新的主回调
    callback_manager.count = 0;
    if (callback && callback_manager.count < RC_MAX_CALLBACKS) {
        callback_manager.callbacks[callback_manager.count++] = callback;
    }
}

void rc_key_add_callback(rc_key_callback_t callback) {
    // 添加额外回调，不清理现有回调
    if (callback && callback_manager.count < RC_MAX_CALLBACKS) {
        callback_manager.callbacks[callback_manager.count++] = callback;
    }
}

void rc_key_remove_callback(rc_key_callback_t callback) {
    // 查找并移除指定回调
    for (int i = 0; i < callback_manager.count; i++) {
        if (callback_manager.callbacks[i] == callback) {
            // 将后面的回调前移
            for (int j = i; j < callback_manager.count - 1; j++) {
                callback_manager.callbacks[j] = callback_manager.callbacks[j + 1];
            }
            callback_manager.count--;
            break;
        }
    }
}

void rc_key_clear_callbacks(void) {
    callback_manager.count = 0;
}

uint8_t rc_key_get_callback_count(void) {
    return callback_manager.count;
}

static void set_event(rc_key_state_t* key, rc_key_mode_t mode, uint32_t deadline) {
    key->pending_event = mode;
    key->event_deadline = deadline;
}

static void clear_event(rc_key_state_t* key) {
    key->pending_event = RC_KEY_NONE;
    key->event_deadline = 0;
    key->operation_done = true;
}

void rc_key_process_events(void) {
    uint32_t now = bsp::GetHighresTickMilliSec();

    for (int i = 0; i < rc_key_count; i++) {
        rc_key_state_t* key = &rc_keys[i];

        if (key->pending_event != RC_KEY_NONE && now >= key->event_deadline) {
            // 调用所有注册的回调函数
            for (int j = 0; j < callback_manager.count; j++) {
                if (callback_manager.callbacks[j]) {
                    callback_manager.callbacks[j](key->config.channel, key->pending_event);
                }
            }
            clear_event(key);
        }
    }
}

void rc_key_process(uint8_t channel, uint16_t value) {
    rc_key_state_t* key = nullptr;

    // 查找对应通道
    for (int i = 0; i < rc_key_count; i++) {
        if (rc_keys[i].config.channel == channel) {
            key = &rc_keys[i];
            break;
        }
    }

    if (!key) return;

    uint32_t now = bsp::GetHighresTickMilliSec();
    bool pressed = (value > key->config.press_thres);

    // 处理间隔控制
    if ((now - key->last_process) < RC_PROCESS_INTERVAL) {
        return;
    }
    key->last_process = now;

    // 重置完成标志
    if (key->operation_done && key->state == RC_KEY_STATE_IDLE) {
        key->operation_done = false;
    }

    // 状态机处理
    switch (key->state) {
        case RC_KEY_STATE_IDLE:
            if (pressed) {
                key->state = RC_KEY_STATE_PRESSED;
                key->press_time = now;
                key->is_pressed = true;
                clear_event(key);
            }
            break;

        case RC_KEY_STATE_PRESSED:
            if (!pressed) {
                key->state = RC_KEY_STATE_RELEASE_WAIT;
                key->release_time = now;
                key->is_pressed = false;

                uint32_t duration = now - key->press_time;

                if (duration >= key->config.long_press_ms) {
                    // 调用所有回调处理长按事件
                    for (int j = 0; j < callback_manager.count; j++) {
                        if (callback_manager.callbacks[j]) {
                            callback_manager.callbacks[j](channel, RC_KEY_LONG_PRESS);
                        }
                    }
                    set_event(key, RC_KEY_RELEASE, now + 10);
                } else {
                    set_event(key, RC_KEY_CLICK, now + key->config.double_click_ms);
                }
            } else if ((now - key->press_time) >= key->config.long_press_ms) {
                if (key->pending_event == RC_KEY_NONE) {
                    // 调用所有回调处理长按事件
                    for (int j = 0; j < callback_manager.count; j++) {
                        if (callback_manager.callbacks[j]) {
                            callback_manager.callbacks[j](channel, RC_KEY_LONG_PRESS);
                        }
                    }
                    set_event(key, RC_KEY_RELEASE, now + key->config.long_press_ms + 100);
                }
            }
            break;

        case RC_KEY_STATE_RELEASE_WAIT:
            if (pressed) {
                key->state = RC_KEY_STATE_PRESSED;
                key->press_time = now;
                key->is_pressed = true;
                clear_event(key);

                // 调用所有回调处理双击事件
                for (int j = 0; j < callback_manager.count; j++) {
                    if (callback_manager.callbacks[j]) {
                        callback_manager.callbacks[j](channel, RC_KEY_DOUBLE_CLICK);
                    }
                }
                set_event(key, RC_KEY_RELEASE, now + 100);
            } else if (now - key->release_time >= key->config.double_click_ms) {
                key->state = RC_KEY_STATE_IDLE;
                if (key->pending_event == RC_KEY_CLICK) {
                    // 调用所有回调处理单击事件
                    for (int j = 0; j < callback_manager.count; j++) {
                        if (callback_manager.callbacks[j]) {
                            callback_manager.callbacks[j](channel, RC_KEY_CLICK);
                        }
                    }
                    clear_event(key);
                }
            }
            break;
    }
}
