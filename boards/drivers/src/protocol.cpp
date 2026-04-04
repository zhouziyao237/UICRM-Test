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

#include "protocol.h"

#include <cstring>
#include <memory>

#include "bsp_usb.h"
#include "crc_check.h"

static const uint8_t SOF = 0xA5;
static const int FRAME_HEADER_LEN = 5;
static const int CMD_ID_LEN = 2;
static const int FRAME_TAIL_LEN = 2;
static const int BYTE = 8;

namespace communication {

    template <typename T>
    static bool CopyExact(T& destination, const uint8_t* data, int length) {
        if (data == nullptr || length != static_cast<int>(sizeof(T)))
            return false;
        memcpy(&destination, data, sizeof(T));
        return true;
    }

    bool Protocol::Receive(package_t package) {
        constexpr int MIN_FRAME_LEN = FRAME_HEADER_LEN + CMD_ID_LEN + FRAME_TAIL_LEN;
        if (package.data == nullptr || package.length < MIN_FRAME_LEN ||
            package.length > MAX_FRAME_LEN) {
            return false;
        }

        memcpy(bufferRx, package.data, static_cast<size_t>(package.length));
        bool processed = false;
        int offset = 0;

        while (offset + MIN_FRAME_LEN <= package.length) {
            while (offset < package.length && bufferRx[offset] != SOF) {
                ++offset;
            }
            if (offset + MIN_FRAME_LEN > package.length) {
                break;
            }

            if (!VerifyHeader(bufferRx + offset, FRAME_HEADER_LEN)) {
                ++offset;
                continue;
            }

            const int data_length =
                (static_cast<int>(bufferRx[offset + 2]) << BYTE) | bufferRx[offset + 1];
            const int frame_length = FRAME_HEADER_LEN + CMD_ID_LEN + data_length + FRAME_TAIL_LEN;
            if (frame_length < MIN_FRAME_LEN || frame_length > MAX_FRAME_LEN) {
                ++offset;
                continue;
            }
            if (offset + frame_length > package.length) {
                break;
            }

            if (VerifyFrame(bufferRx + offset, frame_length)) {
                const int cmd_id =
                    (static_cast<int>(bufferRx[offset + FRAME_HEADER_LEN + 1]) << BYTE) |
                    bufferRx[offset + FRAME_HEADER_LEN];
                processed |= ProcessDataRx(
                    cmd_id, bufferRx + offset + FRAME_HEADER_LEN + CMD_ID_LEN, data_length);
            }
            offset += frame_length;
        }

        if (processed) {
            Heartbeat();
        }
        return processed;
    }

    package_t Protocol::Transmit(int cmd_id) {
        bufferTx[0] = SOF;
        int DATA_LENGTH = ProcessDataTx(cmd_id, bufferTx + FRAME_HEADER_LEN + CMD_ID_LEN);
        const int max_data_length = MAX_FRAME_LEN - FRAME_HEADER_LEN - CMD_ID_LEN - FRAME_TAIL_LEN;
        if (DATA_LENGTH < 0 || DATA_LENGTH > max_data_length)
            return package_t{nullptr, 0};
        bufferTx[1] = (uint8_t)((uint32_t)DATA_LENGTH & 0xFF);
        bufferTx[2] = (uint8_t)((uint32_t)DATA_LENGTH >> BYTE);
        bufferTx[3] = (uint32_t)seq++;
        AppendHeader(bufferTx, FRAME_HEADER_LEN);
        bufferTx[5] = (uint8_t)((uint32_t)cmd_id & 0xFF);
        bufferTx[6] = (uint8_t)((uint32_t)cmd_id >> BYTE);
        AppendFrame(bufferTx, FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN);
        return package_t{bufferTx, FRAME_HEADER_LEN + CMD_ID_LEN + DATA_LENGTH + FRAME_TAIL_LEN};
    }

    bool Protocol::VerifyHeader(const uint8_t* data, int length) {
        return verify_crc8_check_sum(data, length);
    }

    bool Protocol::VerifyFrame(const uint8_t* data, int length) {
        return verify_crc16_check_sum(data, length);
    }

    void Protocol::AppendHeader(uint8_t* data, int length) {
        append_crc8_check_sum(data, length);
    }

    void Protocol::AppendFrame(uint8_t* data, int length) {
        append_crc16_check_sum(data, length);
    }

    // ---------------------------------- UARTProtocol Code ---------------------------------

    UARTProtocol::UARTProtocol(bsp::UART* uart) : Protocol() {
        if (uart == nullptr)
            return;
        uart_ = uart;
        uart_->SetupRxData(&read_ptr_, &read_len_);

        uart_->RegisterCallback(CallbackWrapper, this);
        bsp::thread_init_t thread_init = {
            .func = callback_thread_func_,
            .args = this,
            .attr = callback_thread_attr_,
        };
        callback_thread_ = new bsp::EventThread(thread_init);
    }
    void UARTProtocol::CallbackWrapper(void* args) {
        UARTProtocol* uart_protocol_ = reinterpret_cast<UARTProtocol*>(args);
        if (uart_protocol_ != nullptr && uart_protocol_->callback_thread_ != nullptr)
            uart_protocol_->callback_thread_->Set();
    }
    void UARTProtocol::callback_thread_func_(void* args) {
        UARTProtocol* uart_protocol_ = reinterpret_cast<UARTProtocol*>(args);
        uart_protocol_->Receive(
            communication::package_t{uart_protocol_->read_ptr_, (int)uart_protocol_->read_len_});
    }
    UARTProtocol::~UARTProtocol() {
        if (uart_ != nullptr) {
            uart_->RegisterCallback(nullptr, nullptr);
            uart_->SetupRxData(nullptr, nullptr);
        }
        delete callback_thread_;
    }
    package_t UARTProtocol::Transmit(int cmd_id) {
        package_t package = Protocol::Transmit(cmd_id);
        if (uart_ != nullptr && package.data != nullptr && package.length > 0)
            uart_->Write(package.data, package.length);
        return package;
    }

    // 为抽象类添加默认实现
    bool UARTProtocol::ProcessDataRx(int cmd_id, const uint8_t* data, int length) {
        UNUSED(cmd_id);
        UNUSED(data);
        UNUSED(length);
        return false;
    }

    int UARTProtocol::ProcessDataTx(int cmd_id, uint8_t* data) {
        UNUSED(cmd_id);
        UNUSED(data);
        return -1;
    }

    // ---------------------------------- USBProtocol Code ---------------------------------
#ifndef NO_USB
    USBProtocol::USBProtocol(bsp::VirtualUSB* USB) : Protocol() {
        if (USB == nullptr)
            return;
        usb_ = USB;
        usb_->SetupRxData(&read_ptr_, &read_len_);
        usb_->RegisterCallback(CallbackWrapper, this);
        bsp::thread_init_t thread_init = {
            .func = callback_thread_func_,
            .args = this,
            .attr = callback_thread_attr_,
        };
        callback_thread_ = new bsp::EventThread(thread_init);
    }
    void USBProtocol::CallbackWrapper(void* args) {
        USBProtocol* USB_protocol_ = reinterpret_cast<USBProtocol*>(args);
        if (USB_protocol_ != nullptr && USB_protocol_->callback_thread_ != nullptr)
            USB_protocol_->callback_thread_->Set();
    }
    void USBProtocol::callback_thread_func_(void* args) {
        USBProtocol* USB_protocol_ = reinterpret_cast<USBProtocol*>(args);
        USB_protocol_->Receive(
            communication::package_t{USB_protocol_->read_ptr_, (int)USB_protocol_->read_len_});
    }
    USBProtocol::~USBProtocol() {
        if (usb_ != nullptr) {
            usb_->RegisterCallback(nullptr, nullptr);
            usb_->SetupRxData(nullptr, nullptr);
        }
        delete callback_thread_;
    }
    package_t USBProtocol::Transmit(int cmd_id) {
        package_t package = Protocol::Transmit(cmd_id);
        if (usb_ != nullptr && package.data != nullptr && package.length > 0)
            usb_->Write(package.data, package.length);
        return package;
    }

    // 为抽象类添加默认实现
    bool USBProtocol::ProcessDataRx(int cmd_id, const uint8_t* data, int length) {
        UNUSED(cmd_id);
        UNUSED(data);
        UNUSED(length);
        return false;
    }

    int USBProtocol::ProcessDataTx(int cmd_id, uint8_t* data) {
        UNUSED(cmd_id);
        UNUSED(data);
        return -1;
    }
#endif

    // ----------------------------- ProtocolAdapterManager Code ---------------------------

    // 构造函数
    ProtocolAdapterManager::ProtocolAdapterManager(Protocol_t protocol_t) {
        if (protocol_t.uart) {
            uart_protocol_ = std::make_unique<UARTProtocol>(protocol_t.uart);
            // 注册UART回调
            protocol_t.uart->RegisterCallback(UARTCallbackWrapper, this);

            // 创建UART回调线程
            bsp::thread_init_t thread_init = {
                .func = uart_callback_thread_func_,
                .args = this,
                .attr = callback_thread_attr_,
            };
            uart_callback_thread_ = new bsp::EventThread(thread_init);
        }

#ifndef NO_USB
        if (protocol_t.usb) {
            usb_protocol_ = std::make_unique<USBProtocol>(protocol_t.usb);
            // 注册USB回调
            protocol_t.usb->RegisterCallback(USBCallbackWrapper, this);

            // 创建USB回调线程
            bsp::thread_init_t thread_init = {
                .func = usb_callback_thread_func_,
                .args = this,
                .attr = callback_thread_attr_,
            };
            usb_callback_thread_ = new bsp::EventThread(thread_init);
        }
#endif

        if (protocol_t.uart) {
            protocoltype = UART_ONLY;
#ifndef NO_USB
        } else if (protocol_t.usb) {
            protocoltype = USB_ONLY;
#endif
        } else {
            protocoltype = NONE;
        }
    }

    // 析构函数
    ProtocolAdapterManager::~ProtocolAdapterManager() {
        if (uart_callback_thread_) {
            delete uart_callback_thread_;
        }
#ifndef NO_USB
        if (usb_callback_thread_) {
            delete usb_callback_thread_;
        }
#endif
    }

    // 统一传输接口
    package_t ProtocolAdapterManager::Transmit(Protocol* protocol, int cmd_id) {
        if (protocol == nullptr) {
            return package_t{nullptr, 0};
        }

        package_t package = protocol->Protocol::Transmit(cmd_id);
        if (package.data == nullptr || package.length <= 0) {
            return package;
        }

        switch (protocoltype) {
            case UART_ONLY:
                if (uart_protocol_ && uart_protocol_->uart_ != nullptr) {
                    uart_protocol_->uart_->Write(package.data, package.length);
                }
                break;
#ifndef NO_USB
            case USB_ONLY:
                if (usb_protocol_ && usb_protocol_->usb_ != nullptr) {
                    usb_protocol_->usb_->Write(package.data, package.length);
                }
                break;
#endif
            default:
                break;
        }

        return package;
    }

    void ProtocolAdapterManager::SetReceiver(Protocol* protocol) {
        receiver_ = protocol;
    }

    // 回调包装器实现
    void ProtocolAdapterManager::UARTCallbackWrapper(void* args) {
        ProtocolAdapterManager* adapter = reinterpret_cast<ProtocolAdapterManager*>(args);
        if (adapter->uart_callback_thread_) {
            adapter->uart_callback_thread_->Set();
        }
    }

#ifndef NO_USB
    void ProtocolAdapterManager::USBCallbackWrapper(void* args) {
        ProtocolAdapterManager* adapter = reinterpret_cast<ProtocolAdapterManager*>(args);
        if (adapter->usb_callback_thread_) {
            adapter->usb_callback_thread_->Set();
        }
    }
#endif

    // 回调线程函数实现
    void ProtocolAdapterManager::uart_callback_thread_func_(void* args) {
        ProtocolAdapterManager* adapter = reinterpret_cast<ProtocolAdapterManager*>(args);
        if (adapter->uart_protocol_) {
            // 直接使用读取指针和长度，而不是尝试获取
            package_t package{adapter->uart_protocol_->read_ptr_, (int)adapter->uart_protocol_->read_len_};
            adapter->HandleReceivedData(package, UART_ONLY);
        }
    }

#ifndef NO_USB
    // 修改 usb_callback_thread_func_ 函数
    void ProtocolAdapterManager::usb_callback_thread_func_(void* args) {
        ProtocolAdapterManager* adapter = reinterpret_cast<ProtocolAdapterManager*>(args);
        if (adapter->usb_protocol_) {
            // 直接使用读取指针和长度，而不是尝试获取
            package_t package{adapter->usb_protocol_->read_ptr_, (int)adapter->usb_protocol_->read_len_};
            adapter->HandleReceivedData(package, USB_ONLY);
        }
    }
#endif

    // 统一数据处理入口
    void ProtocolAdapterManager::HandleReceivedData(package_t package, ProtocolType source) {
        UNUSED(source);
        if (receiver_ != nullptr) {
            receiver_->Receive(package);
            return;
        }

        switch (source) {
            case UART_ONLY:
                if (uart_protocol_) {
                    uart_protocol_->Receive(package);
                }
                break;
#ifndef NO_USB
            case USB_ONLY:
                if (usb_protocol_) {
                    usb_protocol_->Receive(package);
                }
                break;
#endif
            default:
                break;
        }
    }

    // ------------------------------------ Referee Code -----------------------------------

    Referee::Referee(bsp::UART* uart) : UARTProtocol(uart) {
        // 设置100ms离线阈值
        SetThreshold(500);
    }

    bool Referee::ProcessDataRx(int cmd_id, const uint8_t* data, int length) {
        switch (cmd_id) {
            case GAME_STATUS:
                if (!CopyExact(game_status, data, length))
                    return false;
                break;
            case GAME_RESULT:
                if (!CopyExact(game_result, data, length))
                    return false;
                break;
            case GAME_ROBOT_HP:
                if (!CopyExact(game_robot_HP, data, length))
                    return false;
                break;
            case EVENT_DATA:
                if (!CopyExact(event_data, data, length))
                    return false;
                break;
            case SUPPLY_PROJECTILE_ACTION:
                if (!CopyExact(supply_projectile_action, data, length))
                    return false;
                break;
            case REFEREE_WARNING:
                if (!CopyExact(referee_warning, data, length))
                    return false;
                break;
            case DART_REMAINING_TIME:
                if (!CopyExact(dart_remaining_time, data, length))
                    return false;
                break;
            case GAME_ROBOT_STATUS:
                if (!CopyExact(game_robot_status, data, length))
                    return false;
                break;
            case POWER_HEAT_DATA:
                if (!CopyExact(power_heat_data, data, length))
                    return false;
                break;
            case GAME_ROBOT_POS:
                if (!CopyExact(game_robot_pos, data, length))
                    return false;
                break;
            case BUFF:
                if (!CopyExact(buff, data, length))
                    return false;
                break;
            case AERIAL_ROBOT_ENERGY:
                if (!CopyExact(aerial_robot_energy, data, length))
                    return false;
                break;
            case ROBOT_HURT:
                if (!CopyExact(robot_hurt, data, length))
                    return false;
                break;
            case SHOOT_DATA:
                if (!CopyExact(shoot_data, data, length))
                    return false;
                break;
            case BULLET_REMAINING:
                if (!CopyExact(bullet_remaining, data, length))
                    return false;
                break;
            case RFID_STATUS:
                if (!CopyExact(rfid_status, data, length))
                    return false;
                break;
            case DART_CLIENT_CMD:
                if (!CopyExact(dart_client_cmd, data, length))
                    return false;
                break;
            case REMOTE_CONTROL_DATA:
                if (!CopyExact(remote_control, data, length))
                    return false;
                break;
            default:
                return false;
        }
        return true;
    }

    int Referee::ProcessDataTx(int cmd_id, uint8_t* data) {
        int data_len;
        switch (cmd_id) {
            case STUDENT_INTERACTIVE: {
                switch (graph_content_) {
                    case NO_GRAPH:
                        data_len = -1;
                        break;
                    case DELETE_GRAPH:
                        data_len = sizeof(graphic_delete_t);
                        memcpy(data, &graphic_delete, data_len);
                        break;
                    case SINGLE_GRAPH:
                        data_len = sizeof(graphic_single_t);
                        memcpy(data, &graphic_single, data_len);
                        break;
                    case DOUBLE_GRAPH:
                        data_len = sizeof(graphic_double_t);
                        memcpy(data, &graphic_double, data_len);
                        break;
                    case FIVE_GRAPH:
                        data_len = sizeof(graphic_five_t);
                        memcpy(data, &graphic_five, data_len);
                        break;
                    case SEVEN_GRAPH:
                        data_len = sizeof(graphic_seven_t);
                        memcpy(data, &graphic_seven, data_len);
                        break;
                    case CHAR_GRAPH:
                        data_len = sizeof(graphic_character_t);
                        memcpy(data, &graphic_character, data_len);
                        break;
                    default:
                        data_len = -1;
                }
                graph_content_ = NO_GRAPH;
                break;
            }
            default:
                data_len = -1;
        }
        return data_len;
    }

    void Referee::PrepareUIContent(content graph_content) {
        graph_content_ = graph_content;
    }

    // ------------------------------------ MiniPC Code -----------------------------------

    Host::Host() : Host(Protocol_t{}) {
    }

    Host::Host(bsp::UART* uart)
#ifndef NO_USB
        : Host(Protocol_t{uart, nullptr}) {
#else
        : Host(Protocol_t{uart}) {
#endif
    }

#ifndef NO_USB
    Host::Host(bsp::VirtualUSB* usb) : Host(Protocol_t{nullptr, usb}) {
    }
#endif

    Host::Host(Protocol_t protocol_t) : Protocol(), adapter_(protocol_t) {
        adapter_.SetReceiver(this);
    }

    package_t Host::Transmit(int cmd_id) {
        return adapter_.Transmit(this, cmd_id);
    }

    bool Host::ProcessDataRx(int cmd_id, const uint8_t* data, int length) {
        switch (cmd_id) {
            case PACK:
                if (!CopyExact(pack, data, length))
                    return false;
                break;
            case TARGET_ANGLE:
                // memset(&target_angle, 0, sizeof(target_angle));
                if (!CopyExact(target_angle, data, length))
                    return false;
                break;
            case NO_TARGET_FLAG:
                if (!CopyExact(no_target_flag, data, length))
                    return false;
                break;
            case SHOOT_CMD:
                if (!CopyExact(shoot_cmd, data, length))
                    return false;
                break;
            case ROBOT_MOVE_SPEED:
                if (!CopyExact(robot_move, data, length))
                    return false;
                break;
            case ROBOT_POWER_HEAT_HP_UPLOAD:
                if (!CopyExact(robot_power_heat_hp_upload, data, length))
                    return false;
                break;
            case GIMBAL_CURRENT_STATUS:
                if (!CopyExact(gimbal_current_status, data, length))
                    return false;
                break;
            case CHASSIS_CURRENT_STATUS:
                if (!CopyExact(chassis_current_status, data, length))
                    return false;
                break;
            case AUTOAIM_ENABLE:
                if (!CopyExact(autoaim_enable, data, length))
                    return false;
                break;
            case ROBOT_STATUS_UPLOAD:
                if (!CopyExact(robot_status_upload, data, length))
                    return false;
                break;
            default:
                return false;
        }
        return true;
    }

    int Host::ProcessDataTx(int cmd_id, uint8_t* data) {
        int data_len;
        switch (cmd_id) {
            case PACK:
                data_len = sizeof(pack_t);
                memcpy(data, &pack, data_len);
                break;
            case TARGET_ANGLE:
                data_len = sizeof(target_angle_t);
                memcpy(data, &target_angle, data_len);
                break;
            case NO_TARGET_FLAG:
                data_len = sizeof(no_target_flag_t);
                memcpy(data, &no_target_flag, data_len);
                break;
            case SHOOT_CMD:
                data_len = sizeof(shoot_cmd_t);
                memcpy(data, &shoot_cmd, data_len);
                break;
            case ROBOT_MOVE_SPEED:
                data_len = sizeof(robot_move_t);
                memcpy(data, &robot_move, data_len);
                break;
            case ROBOT_POWER_HEAT_HP_UPLOAD:
                data_len = sizeof(robot_power_heat_hp_upload_t);
                memcpy(data, &robot_power_heat_hp_upload, data_len);
                break;
            case GIMBAL_CURRENT_STATUS:
                data_len = sizeof(gimbal_current_status_t);
                memcpy(data, &gimbal_current_status, data_len);
                break;
            case CHASSIS_CURRENT_STATUS:
                data_len = sizeof(chassis_current_status_t);
                memcpy(data, &chassis_current_status, data_len);
                break;
            case AUTOAIM_ENABLE:
                data_len = sizeof(autoaim_enable_t);
                memcpy(data, &autoaim_enable, data_len);
                break;
            case ROBOT_STATUS_UPLOAD:
                data_len = sizeof(robot_status_upload_t);
                memcpy(data, &robot_status_upload, data_len);
                break;
            default:
                data_len = -1;
        }
        return data_len;
    }

} /* namespace communication */
