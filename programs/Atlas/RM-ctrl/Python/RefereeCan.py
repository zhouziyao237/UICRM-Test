import struct
from typing import Callable, Dict, Any

# ===================== 1. 复用原有裁判系统定义 =====================
class RefereeCmdId:
    ROBOT_STATE = 0x0001    # 机器人状态
    GAME_STATUS = 0x0002    # 游戏状态
    BULLET_INFO = 0x0003    # 子弹信息

class RobotStateData:
    def __init__(self):
        self.robot_speed: float = 0.0    # 4字节
        self.robot_hp: int = 0           # 2字节
        self.robot_status: int = 0       # 1字节
        self.bullet_num: int = 0         # 1字节
        self.timestamp: int = 0          # 4字节
        self.target_angle: float = 0.0   # 4字节

    def to_bytes(self) -> bytes:
        # 严格固定16字节：f(4)+H(2)+B(1)+B(1)+I(4)+f(4) = 16字节
        return struct.pack(
            "!fHBBIf",
            self.robot_speed,
            self.robot_hp,
            self.robot_status,
            self.bullet_num,
            self.timestamp,
            self.target_angle
        )

    def from_bytes(self, data: bytes) -> bool:
        if len(data) != 16:
            return False
        unpacked = struct.unpack("!fHBBIf", data)
        self.robot_speed = unpacked[0]
        self.robot_hp = unpacked[1]
        self.robot_status = unpacked[2]
        self.bullet_num = unpacked[3]
        self.timestamp = unpacked[4]
        self.target_angle = unpacked[5]
        return True

    def __repr__(self) -> str:
        return (f"RobotStateData(speed={self.robot_speed:.2f}, hp={self.robot_hp}, "
                f"status={self.robot_status}, bullet={self.bullet_num}, "
                f"timestamp={self.timestamp}, angle={self.target_angle:.2f})")

# ===================== 2. CAN Bridge抽象层 =====================
class CanFrame:
    def __init__(self):
        self.std_id: int = 0
        self.data: bytes = b""
        self.dlc: int = 8

class CanBridge:
    def __init__(self):
        self.tx_frames: list[CanFrame] = []
        self.rx_callback: Callable[[CanFrame], None] = None

    def send(self, frame: CanFrame) -> bool:
        if len(frame.data) > 8:
            return False
        frame.data = frame.data.ljust(8, b'\x00')  # 补全8字节
        self.tx_frames.append(frame)
        print(f"[CAN Bridge] 发送帧 ID={hex(frame.std_id)} 数据={frame.data.hex()}")
        return True

    def register_rx_callback(self, callback: Callable[[CanFrame], None]):
        self.rx_callback = callback

    def simulate_transmit(self):
        if self.rx_callback:
            for frame in self.tx_frames:
                self.rx_callback(frame)
        self.tx_frames.clear()

# ===================== 3. 裁判CAN协议类（修正核心逻辑） =====================
class RefereeCanProtocol:
    DECLARE_ID_REFEREE = 0x01
    MAX_REF_DATA_LEN = 128
    REF_CAN_BASE_ID = 0x200

    def __init__(self, can_bridge: CanBridge):
        self.can_bridge = can_bridge
        self.can_bridge.register_rx_callback(self._on_can_frame_received)

        self.rx_buffer: bytes = b""
        self.rx_header: dict = {}
        self.rx_pack_received: int = 0
        self.robot_state_callback: Callable[[RobotStateData], None] = None

    # ------------------- 核心工具函数（修正） -------------------
    def _calc_crc8(self, data: bytes) -> int:
        """标准CRC8 (poly=0x07) 计算"""
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc = crc << 1
                crc &= 0xFF  # 强制8位，避免溢出
        return crc

    def _send_can_frame(self, std_id: int, data: bytes) -> bool:
        frame = CanFrame()
        frame.std_id = std_id
        frame.data = data[:8]  # 强制截断为8字节
        return self.can_bridge.send(frame)

    # ------------------- 发送逻辑（修正头帧格式+分包） -------------------
    def send_robot_state(self, data: RobotStateData) -> bool:
        data_bytes = data.to_bytes()
        return self.send_data(RefereeCmdId.ROBOT_STATE, data_bytes)

    def send_data(self, cmd_id: RefereeCmdId, data: bytes) -> bool:
        if len(data) > self.MAX_REF_DATA_LEN:
            return False

        # 1. 计算分包数（6字节/帧）
        data_len = len(data)
        total_pack = (data_len + 5) // 6  # 向上取整：16字节 → 3帧（6+6+4）
        print(f"[发送准备] 数据长度={data_len} 总包数={total_pack}")

        # 2. 发送头帧（严格8字节格式）
        # 头帧格式：[0:1]声明ID, [1:3]cmd_id, [3:4]总包数, [4:8]数据长度
        header_data = struct.pack(
            "!BHB I",  # 空格仅为可读性，实际是 BHBI（1+2+1+4=8字节）
            self.DECLARE_ID_REFEREE,
            cmd_id,
            total_pack,
            data_len
        )
        self._send_can_frame(self.REF_CAN_BASE_ID + 0, header_data)

        # 3. 发送数据帧（修正：按实际长度截取，避免多余填充）
        for pack_idx in range(total_pack):
            start = pack_idx * 6
            end = start + 6
            pack_data = data[start:end]  # 最后一帧自动截断为实际长度（如4字节）
            # 数据帧格式：[0:1]声明ID, [1:2]包序号, [2:8]数据（最多6字节）
            data_frame = bytes([self.DECLARE_ID_REFEREE, pack_idx]) + pack_data
            self._send_can_frame(self.REF_CAN_BASE_ID + 1, data_frame)

        # 4. 发送尾帧（严格8字节格式）
        # 尾帧格式：[0:1]声明ID, [1:2]CRC8, [2:4]cmd_id, [4:5]结束标识, [5:8]预留
        crc8 = self._calc_crc8(data)
        tail_data = struct.pack(
            "!BBH B",  # 1+1+2+1=5字节，剩余3字节预留
            self.DECLARE_ID_REFEREE,
            crc8,
            cmd_id,
            0xAA  # 结束标识
        )
        self._send_can_frame(self.REF_CAN_BASE_ID + 2, tail_data)

        return True

    # ------------------- 接收逻辑（修正解包顺序） -------------------
    def _on_can_frame_received(self, frame: CanFrame):
        # 基础过滤
        if len(frame.data) < 1 or frame.data[0] != self.DECLARE_ID_REFEREE:
            return
        if not (self.REF_CAN_BASE_ID <= frame.std_id <= self.REF_CAN_BASE_ID + 2):
            return

        print(f"[RefereeProtocol] 接收帧 ID={hex(frame.std_id)} 数据={frame.data.hex()}")

        # 1. 解析头帧（修正解包顺序）
        if frame.std_id == self.REF_CAN_BASE_ID + 0:
            # 严格按发送格式解析：B(1) + H(2) + B(1) + I(4)
            declare_id = frame.data[0]
            cmd_id = struct.unpack("!H", frame.data[1:3])[0]
            total_pack = frame.data[3]
            data_len = struct.unpack("!I", frame.data[4:8])[0]

            self.rx_header = {
                "cmd_id": cmd_id,
                "total_pack": total_pack,
                "data_len": data_len
            }
            self.rx_buffer = b""
            self.rx_pack_received = 0

            print(f"[RefereeProtocol] 解析头帧 "
                  f"cmd_id={hex(cmd_id)} total_pack={total_pack} data_len={data_len}")

        # 2. 解析数据帧（拼包，修正截断）
        elif frame.std_id == self.REF_CAN_BASE_ID + 1:
            pack_idx = frame.data[1]
            total_pack = self.rx_header.get("total_pack", 0)

            if 0 <= pack_idx < total_pack:
                # 计算当前帧应取的实际长度
                remaining_len = self.rx_header["data_len"] - pack_idx * 6
                take_len = min(6, remaining_len)
                pack_data = frame.data[2:2+take_len]  # 仅取有效长度，忽略填充0

                self.rx_buffer += pack_data
                self.rx_pack_received += 1
                print(f"[RefereeProtocol] 解析数据帧 序号={pack_idx} "
                      f"已接收={self.rx_pack_received}/{total_pack} 数据={pack_data.hex()}")

        # 3. 解析尾帧（修正校验逻辑）
        elif frame.std_id == self.REF_CAN_BASE_ID + 2:
            # 解析尾帧字段
            crc8 = frame.data[1]
            cmd_id = struct.unpack("!H", frame.data[2:4])[0]
            end_flag = frame.data[4]

            # 校验条件（全部满足才通过）
            conditions = [
                end_flag == 0xAA,
                cmd_id == self.rx_header.get("cmd_id", 0),
                self.rx_pack_received == self.rx_header.get("total_pack", 0),
                len(self.rx_buffer) == self.rx_header.get("data_len", 0),
                self._calc_crc8(self.rx_buffer) == crc8
            ]

            if all(conditions):
                print(f"[RefereeProtocol] 尾帧校验通过！CRC8=0x{crc8:02x}")
                # 触发回调
                if self.rx_header["cmd_id"] == RefereeCmdId.ROBOT_STATE and self.robot_state_callback:
                    robot_data = RobotStateData()
                    if robot_data.from_bytes(self.rx_buffer):
                        self.robot_state_callback(robot_data)
            else:
                print(f"[RefereeProtocol] 尾帧校验失败！")
                print(f"  - 结束标识: {end_flag == 0xAA} (实际0x{end_flag:02x})")
                print(f"  - CMD_ID匹配: {cmd_id == self.rx_header.get('cmd_id', 0)} (接收0x{cmd_id:04x}, 期望0x{self.rx_header.get('cmd_id', 0):04x})")
                print(f"  - 分包数匹配: {self.rx_pack_received == self.rx_header.get('total_pack', 0)} (接收{self.rx_pack_received}, 期望{self.rx_header.get('total_pack', 0)})")
                print(f"  - 数据长度匹配: {len(self.rx_buffer) == self.rx_header.get('data_len', 0)} (接收{len(self.rx_buffer)}, 期望{self.rx_header.get('data_len', 0)})")
                print(f"  - CRC8匹配: {self._calc_crc8(self.rx_buffer) == crc8} (计算0x{self._calc_crc8(self.rx_buffer):02x}, 接收0x{crc8:02x})")

    def register_robot_state_callback(self, callback: Callable[[RobotStateData], None]):
        self.robot_state_callback = callback

# ===================== 4. 测试主逻辑 =====================
def test_referee_can_communication():
    print("===== 裁判系统CAN Bridge算法测试（修正版） =====")

    # 1. 初始化组件
    can_bridge = CanBridge()
    ref_protocol = RefereeCanProtocol(can_bridge)

    # 2. 注册回调
    def on_robot_state_received(data: RobotStateData):
        print(f"\n[业务层] 成功接收并解析机器人状态：\n{data}")

    ref_protocol.register_robot_state_callback(on_robot_state_received)

    # 3. 构造测试数据
    send_data = RobotStateData()
    send_data.robot_speed = 3.5
    send_data.robot_hp = 100
    send_data.robot_status = 0x01
    send_data.bullet_num = 50
    send_data.timestamp = 123456
    send_data.target_angle = 45.8

    print(f"\n[业务层] 准备发送机器人状态：\n{send_data}")
    print(f"[业务层] 序列化后字节长度={len(send_data.to_bytes())} 字节={send_data.to_bytes().hex()}")

    # 4. 发送数据
    print("\n----- 发送阶段 -----")
    ref_protocol.send_robot_state(send_data)

    # 5. 模拟总线传输
    print("\n----- 模拟总线传输 -----")
    can_bridge.simulate_transmit()

if __name__ == "__main__":
    test_referee_can_communication()