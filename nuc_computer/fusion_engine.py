"""
Sensor fusion engine
Intel NUC side — EKF + complementary filter
Inputs: STM32 IMU frames, camera odometry, RTK lat/lon
Output: fused 6-DOF pose + 3D rail map
"""

import time
import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class Pose6D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z,
                         self.roll, self.pitch, self.yaw])


@dataclass
class RailPoint:
    x: float; y: float; z: float
    gauge: float = 1435.0
    wear: float = 0.0


class EKFFusion:
    """Lightweight EKF for rail inspection vehicle"""

    def __init__(self):
        self.pose = Pose6D()
        # covariance
        self.P = np.eye(6) * 0.1
        self.Q_imu = np.eye(6) * 0.001
        self.R_lidar = np.eye(3) * 0.005
        self.R_rtk = np.eye(3) * 0.01
        self.rail_map: List[RailPoint] = []

        # IMU bias estimates
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)

        self._last_t = time.time()
        self._frame_id = 0

    def predict_imu(self, accel: np.ndarray, gyro: np.ndarray, dt: float):
        """EKF predict step: IMU integration"""
        g_clean = gyro - self.gyro_bias
        a_clean = accel - self.accel_bias

        self.pose.roll += g_clean[0] * dt
        self.pose.pitch += g_clean[1] * dt
        self.pose.yaw += g_clean[2] * dt

        cy = math.cos(self.pose.yaw)
        sy = math.sin(self.pose.yaw)
        self.pose.x += (a_clean[0] * cy - a_clean[1] * sy) * dt * dt * 0.5
        self.pose.y += (a_clean[0] * sy + a_clean[1] * cy) * dt * dt * 0.5

        self.P = self.P + self.Q_imu * dt

    def update_rtk(self, pos: np.ndarray):
        """EKF update: RTK GNSS measurement"""
        z = pos[:3]
        h = self.pose.array()[:3]
        y = z - h
        # Simplified: treat as direct observation
        alpha = 0.3
        self.pose.x += alpha * y[0]
        self.pose.y += alpha * y[1]
        self.pose.z += alpha * y[2]
        self.P[:3, :3] *= (1 - alpha)

    def complementary_filter(self, accel: np.ndarray, dt: float):
        """Accelerometer tilt for roll/pitch (prevents gyro drift)"""
        ax, ay, az = accel[0], accel[1], accel[2]
        norm = math.sqrt(ax*ax + ay*ay + az*az) + 1e-6
        a_roll = math.atan2(ay, az)
        a_pitch = math.atan2(-ax, norm)
        alpha = 0.98
        self.pose.roll = alpha * self.pose.roll + (1 - alpha) * a_roll
        self.pose.pitch = alpha * self.pose.pitch + (1 - alpha) * a_pitch

    def update(self, stm32_frame: any) -> dict:
        """Process one STM32 frame: predict + correct"""
        now = time.time()
        dt = now - self._last_t
        if dt <= 0.0:
            dt = 0.01
        self._last_t = now
        self._frame_id += 1

        try:
            accel = np.array([stm32_frame.accel_x,
                              stm32_frame.accel_y,
                              stm32_frame.accel_z])
            gyro = np.array([stm32_frame.gyro_x,
                             stm32_frame.gyro_y,
                             stm32_frame.gyro_z])

            self.predict_imu(accel, gyro, dt)
            self.complementary_filter(accel, dt)

        except AttributeError:
            pass

        return {
            "frame_id": self._frame_id,
            "pose": self.pose.array().tolist(),
            "rail_pts": len(self.rail_map),
            "yaw_deg": math.degrees(self.pose.yaw),
        }

    def obstacle_distance(self, ultrasonic: List[float]) -> float:
        """Minimum obstacle range (m)"""
        valid = [d for d in ultrasonic if d > 0]
        return min(valid) if valid else 99.0

    def should_brake(self, distance_m: float) -> int:
        """Multi-level braking decision"""
        if distance_m < 0.5:
            return 3   # emergency stop
        elif distance_m < 2.0:
            return 2   # creep
        elif distance_m < 5.0:
            return 1   # slow
        return 0        # cruise
