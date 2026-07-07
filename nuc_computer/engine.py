"""
NUC main orchestrator
Reads STM32 frames → fusion → YOLOv8 detection → upload to cloud
"""

import time
import json
import threading
from pathlib import Path

import cv2
import numpy as np
import requests

from config import UPPER_COMPUTER_URL, UPLOAD_INTERVAL, CAMERA_INDEX
from stm32_serial import STM32SerialReader, STM32Frame
from defect_detector import DefectDetector, ThermalReader
from fusion_engine import EKFFusion


class RailInspectEngine:
    """Main engine running on Intel NUC"""

    def __init__(self):
        self.stm32 = STM32SerialReader()
        self.detector = DefectDetector()
        self.thermal = ThermalReader()
        self.fusion = EKFFusion()

        self.cap: cv2.VideoCapture = None
        self._running = False
        self._last_upload = 0.0
        self._defects_today: list = []
        self._stats = {
            "frames": 0, "detections": 0, "alerts": 0,
            "emergency_stops": 0,
        }

    def start(self):
        if self.stm32.connect():
            self.stm32.start()
            print("[Engine] STM32 connected, reading frames")
        else:
            print("[Engine] STM32 not available, running in simulation")

        if CAMERA_INDEX >= 0:
            self.cap = cv2.VideoCapture(CAMERA_INDEX)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            if not self.cap.isOpened():
                print("[Engine] camera unavailable, skipping vision")
                self.cap = None

        self._running = True
        t = threading.Thread(target=self._loop, daemon=True, name="Engine")
        t.start()
        print("[Engine] main loop started")
        return self

    def stop(self):
        self._running = False
        if self.stm32:
            self.stm32.stop()
        if self.cap:
            self.cap.release()

    def get_stats(self) -> dict:
        return dict(self._stats)

    def _loop(self):
        while self._running:
            cycle_start = time.time()

            # 1. read STM32 sensor frame
            frame = self.stm32.get_frame()
            if frame is None:
                frame = self._mock_frame()

            self._stats["frames"] += 1

            # 2. sensor fusion
            fusion_out = self.fusion.update(frame)

            # 3. capture camera image
            img = None
            if self.cap and self.cap.isOpened():
                ok, img = self.cap.read()
                if not ok:
                    img = None

            # 4. YOLOv8 defect detection
            defects = []
            if img is not None:
                t_max = self.thermal.read_max_temp()
                defects = self.detector.detect(img, t_max)

            for d in defects:
                self._defects_today.append(d.to_dict())
                self._stats["detections"] += 1
                if d.alert_level >= 3:
                    self._stats["alerts"] += 1
                    print(f"[ALERT] L3 defect: {d.label} "
                          f"conf={d.confidence:.2f} "
                          f"size={d.size_mm}")

            # 5. obstacle braking decision
            min_dist = self.fusion.obstacle_distance(frame.obstacles)
            brake = self.fusion.should_brake(min_dist)
            if brake >= 3:
                self.stm32.send_command(brake_level=3)
                self._stats["emergency_stops"] += 1
                print(f"[SAFETY] emergency brake! dist={min_dist:.2f}m")
            elif brake >= 1:
                speed = {1: 0.3, 2: 0.1}.get(brake, 0.5)
                self.stm32.send_command(brake_level=brake, speed=speed)

            # 6. periodic upload to cloud
            now = time.time()
            if now - self._last_upload >= UPLOAD_INTERVAL:
                self._upload_frame(frame, fusion_out, defects)
                self._last_upload = now
                self._defects_today.clear()

            # 7. rate control (~5 Hz with vision, 20 Hz without)
            elapsed = time.time() - cycle_start
            target = 0.05 if img is not None else 0.2
            if elapsed < target:
                time.sleep(target - elapsed)

    def _upload_frame(self, frame: STM32Frame, fusion: dict,
                      defects: list):
        """Push one aggregated frame to cloud"""
        payload = {
            "frame_id": frame.frame_id,
            "timestamp_ms": frame.timestamp_ms,
            "state": frame.robot_state,
            "battery": {"v": frame.battery_v, "soc": frame.battery_soc},
            "pose": fusion.get("pose", [0]*6),
            "yaw_deg": fusion.get("yaw_deg", 0),
            "speed": 0.5,
            "rail_temp": frame.rail_temp,
            "alert_flags": frame.alert_flags,
            "obstacles": frame.obstacles,
            "defects": [d.to_dict() for d in defects] if defects else [],
        }
        try:
            r = requests.post(f"{UPPER_COMPUTER_URL}/api/ingest",
                              json=payload, timeout=2)
        except requests.RequestException:
            pass  # offline cache would go here

    def _mock_frame(self) -> STM32Frame:
        """Generate simulated frame when STM32 is offline"""
        f = STM32Frame()
        f.frame_id = self._stats["frames"]
        f.timestamp_ms = int(time.time() * 1000)
        import random
        f.accel_x = random.gauss(0, 0.05)
        f.accel_y = random.gauss(0, 0.05)
        f.accel_z = 9.81 + random.gauss(0, 0.02)
        f.gyro_z = random.gauss(0, 0.005)
        f.battery_v = 46.5 - self._stats["frames"] * 0.0005
        f.battery_soc = int((f.battery_v - 36.0) / 12.0 * 100)
        f.robot_state = 3
        f.ultrasonic = [200 + i * 100 + random.randint(-50, 50)
                        for i in range(8)]
        return f


# ═══ entry ═══

_engine: RailInspectEngine = None

def get_engine() -> RailInspectEngine:
    global _engine
    if _engine is None:
        _engine = RailInspectEngine()
    return _engine


if __name__ == "__main__":
    eng = RailInspectEngine().start()
    print("NUC engine running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(2)
            stats = eng.get_stats()
            print(f"  frames={stats['frames']} "
                  f"detects={stats['detections']} "
                  f"alerts={stats['alerts']} "
                  f"stops={stats['emergency_stops']}")
    except KeyboardInterrupt:
        eng.stop()
        print("stopped")
