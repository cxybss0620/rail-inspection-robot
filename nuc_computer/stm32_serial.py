#!/usr/bin/env python3
"""
stm32_serial.py - STM32 串口通信模块
==============================
功能: 从 STM32F407 串口接收传感器数据帧，解析并缓存
协议: 定长二进制帧 + 起始/结束标记

帧格式 (40字节):
  Byte 0:     0xAA  帧头
  Byte 1:     0x55  帧头
  Byte 2-3:   frame_id (uint16, big-endian)
  Byte 4-7:   timestamp_ms (uint32, big-endian)
  Byte 8-9:   accel_x (int16 ×100)
  Byte 10-11: accel_y (int16 ×100)
  Byte 12-13: accel_z (int16 ×100)
  Byte 14-15: gyro_x (int16 ×100)
  Byte 16-17: gyro_y (int16 ×100)
  Byte 18-19: gyro_z (int16 ×100)
  Byte 20:    imu_temp (uint8, +30偏移)
  Byte 21-22: battery_v (uint16, mV)
  Byte 23:    battery_soc (uint8, %)
  Byte 24:    rail_temp (int8, +30偏移)
  Byte 25:    alert_flags (uint8)
  Byte 26:    robot_state (uint8)
  Byte 27-34: ultrasonic[8] (uint8, cm)
  Byte 35-38: crc32 (uint32)
  Byte 39:    0xBB  帧尾
"""

import struct
import time
import threading
from dataclasses import dataclass, field
from typing import Optional, List
import serial
import serial.tools.list_ports

from config import STM32_SERIAL_PORT, STM32_BAUDRATE

FRAME_HEADER = bytes([0xAA, 0x55])
FRAME_FOOTER = 0xBB
FRAME_LENGTH = 40


@dataclass
class STM32Frame:
    """解析后的传感器帧"""
    frame_id: int = 0
    timestamp_ms: int = 0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 9.81
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    imu_temp: float = 25.0
    battery_v: float = 0.0
    battery_soc: int = 0
    rail_temp: float = 25.0
    alert_flags: int = 0
    robot_state: int = 0
    ultrasonic: List[int] = field(default_factory=lambda: [0] * 8)
    crc32: int = 0

    STATE_NAMES = {
        0: "IDLE", 1: "INIT", 2: "READY",
        3: "CRUISING", 4: "INSPECTING", 5: "ALERT",
        6: "EMERGENCY_STOP", 7: "CHARGING", 8: "ERROR",
    }

    @property
    def state_name(self) -> str:
        return self.STATE_NAMES.get(self.robot_state, "UNKNOWN")

    @property
    def obstacles(self) -> List[float]:
        """超声波距离 (m)"""
        return [d / 100.0 for d in self.ultrasonic]

    @property
    def min_obstacle_distance(self) -> float:
        valid = [d for d in self.ultrasonic if d > 0]
        return min(valid) / 100.0 if valid else 99.0


class STM32SerialReader:
    """STM32 串口读取线程"""

    def __init__(self, port: str = STM32_SERIAL_PORT, baudrate: int = STM32_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.latest_frame: Optional[STM32Frame] = None
        self.frame_count = 0
        self.error_count = 0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._buffer = bytearray()

    def connect(self) -> bool:
        """连接 STM32 串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
            )
            print(f"[STM32] 串口已连接: {self.port} @ {self.baudrate}")
            return True
        except serial.SerialException as e:
            print(f"[STM32] 串口连接失败: {e}")
            # 尝试自动查找
            ports = list(serial.tools.list_ports.comports())
            stm32_ports = [p.device for p in ports if "STM" in p.description or "USB" in p.description]
            if stm32_ports:
                print(f"[STM32] 尝试自动端口: {stm32_ports[0]}")
                try:
                    self.ser = serial.Serial(stm32_ports[0], self.baudrate, timeout=0.1)
                    self.port = stm32_ports[0]
                    print(f"[STM32] 自动连接成功: {self.port}")
                    return True
                except:
                    pass
            return False

    def start(self):
        """启动读取线程"""
        if not self.ser:
            if not self.connect():
                raise RuntimeError("无法连接 STM32 串口")
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True, name="STM32-Reader")
        self._thread.start()
        print("[STM32] 读取线程已启动")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.ser:
            self.ser.close()
        print("[STM32] 已停止")

    def get_frame(self) -> Optional[STM32Frame]:
        with self._lock:
            return self.latest_frame

    def _read_loop(self):
        """主循环: 读取串口数据, 解析帧"""
        while self._running:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    self._buffer.extend(data)

                    # 解析帧
                    while len(self._buffer) >= FRAME_LENGTH:
                        # 查找帧头
                        if self._buffer[0] != 0xAA or self._buffer[1] != 0x55:
                            idx = self._buffer.find(FRAME_HEADER)
                            if idx < 0:
                                self._buffer.clear()
                                break
                            self._buffer = self._buffer[idx:]
                            continue

                        # 检查帧尾
                        if self._buffer[FRAME_LENGTH - 1] != FRAME_FOOTER:
                            self._buffer = self._buffer[1:]
                            self.error_count += 1
                            continue

                        # 提取完整帧
                        frame_bytes = bytes(self._buffer[:FRAME_LENGTH])
                        self._buffer = self._buffer[FRAME_LENGTH:]

                        # CRC32 校验
                        payload = frame_bytes[0:35]
                        crc_calc = struct.unpack(">I", frame_bytes[35:39])[0]
                        import zlib
                        if zlib.crc32(payload) & 0xFFFFFFFF != crc_calc:
                            self.error_count += 1
                            continue

                        # 解析
                        frame = self._parse_frame(frame_bytes)
                        if frame:
                            with self._lock:
                                self.latest_frame = frame
                                self.frame_count += 1

                else:
                    time.sleep(0.005)  # 空闲时 5ms 等待

            except serial.SerialException as e:
                print(f"[STM32] 串口错误: {e}")
                self.error_count += 1
                time.sleep(0.5)
                # 尝试重连
                try:
                    self.ser.close()
                    self.ser.open()
                except:
                    pass
            except Exception as e:
                print(f"[STM32] 未知错误: {e}")
                self.error_count += 1

    def _parse_frame(self, data: bytes) -> Optional[STM32Frame]:
        try:
            f = STM32Frame()
            f.frame_id = struct.unpack(">H", data[2:4])[0]
            f.timestamp_ms = struct.unpack(">I", data[4:8])[0]

            def _i16(d): return struct.unpack(">h", d)[0]

            f.accel_x = _i16(data[8:10]) / 100.0
            f.accel_y = _i16(data[10:12]) / 100.0
            f.accel_z = _i16(data[12:14]) / 100.0
            f.gyro_x  = _i16(data[14:16]) / 100.0
            f.gyro_y  = _i16(data[16:18]) / 100.0
            f.gyro_z  = _i16(data[18:20]) / 100.0

            f.imu_temp = data[20] + 30.0
            f.battery_v = struct.unpack(">H", data[21:23])[0] / 1000.0
            f.battery_soc = data[23]
            f.rail_temp = data[24] + 30.0
            f.alert_flags = data[25]
            f.robot_state = data[26]
            f.ultrasonic = [data[27 + i] for i in range(8)]
            f.crc32 = struct.unpack(">I", data[35:39])[0]

            return f
        except Exception as e:
            print(f"[STM32] 解析失败: {e}")
            return None

    def send_command(self, brake_level: int = 0, speed: float = 0.0,
                     steer: float = 0.0, pan: float = 90.0, tilt: float = 90.0):
        """
        向 STM32 发送控制指令
        格式: 0xCC 0xDD brake speed×100(2B) steer×100(2B) pan tilt checksum 0xEE
        """
        if not self.ser or not self.ser.is_open:
            print("[STM32] 无法发送: 串口未连接")
            return

        cmd = bytearray(14)
        cmd[0] = 0xCC
        cmd[1] = 0xDD
        cmd[2] = brake_level & 0x03
        sp = max(-32768, min(32767, int(speed * 100)))
        cmd[3] = (sp >> 8) & 0xFF
        cmd[4] = sp & 0xFF
        st = max(-32768, min(32767, int(steer * 100)))
        cmd[5] = (st >> 8) & 0xFF
        cmd[6] = st & 0xFF
        cmd[7] = max(0, min(180, int(pan)))
        cmd[8] = max(0, min(180, int(tilt)))
        # 预留 9-11
        cmd[12] = 0  # checksum
        for i in range(12):
            cmd[12] ^= cmd[i]
        cmd[13] = 0xEE

        try:
            self.ser.write(cmd)
            self.ser.flush()
        except Exception as e:
            print(f"[STM32] 发送失败: {e}")


# ═══ 单例 ═══
_stm32_reader: Optional[STM32SerialReader] = None

def get_stm32() -> STM32SerialReader:
    global _stm32_reader
    if _stm32_reader is None:
        _stm32_reader = STM32SerialReader()
    return _stm32_reader


# ═══ 测试 ═══
if __name__ == "__main__":
    reader = STM32SerialReader()
    if reader.connect():
        reader.start()
        print("等待数据...")
        for _ in range(10):
            time.sleep(0.5)
            frame = reader.get_frame()
            if frame:
                print(f"帧 {frame.frame_id}: "
                      f"IMU({frame.accel_x:.2f},{frame.accel_y:.2f},{frame.accel_z:.2f}) "
                      f"电池={frame.battery_v:.1f}V SOC={frame.battery_soc}% "
                      f"超声={frame.obstacles}")
        reader.stop()
    else:
        print("模拟模式: 串口不可用，使用模拟数据")
        # 返回模拟数据供测试
