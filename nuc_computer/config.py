#!/usr/bin/env python3
"""
NUC 端 - 系统配置
===============
硬件: Intel NUC / Jetson Xavier, Ubuntu 20.04/22.04
软件: Python 3.8+, OpenCV, PyTorch, pyserial, NumPy
通信: 串口(STM32) + HTTP(上位机)
"""

import os

# ═══ 硬件连接 ═══
# STM32 <-> NUC 串口
STM32_SERIAL_PORT = "/dev/ttyUSB0"
STM32_BAUDRATE = 115200

# ═══ YOLOv8 模型 ═══
YOLO_MODEL_PATH = os.path.join(os.path.dirname(__file__), "models/yolov8n_rail.pt")
YOLO_CONF = 0.5
YOLO_IOU = 0.45
YOLO_IMG_SIZE = 640
DETECT_CLASSES = {
    0: "裂纹",
    1: "剥离掉块",
    2: "压溃",
    3: "波磨",
    4: "扣件松动",
    5: "轨面擦伤",
}

# ═══ 相机 ═══
CAMERA_INDEX = 0            # /dev/video0
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_FPS = 30

# ═══ 热成像 ═══
THERMAL_PORT = "/dev/ttyUSB1"
THERMAL_BAUDRATE = 9600

# ═══ 相机标定参数（需实际标定） ═══
CAMERA_MATRIX = [
    [1660.0, 0, 960.0],
    [0, 1660.0, 540.0],
    [0, 0, 1.0],
]
WORKING_DISTANCE_MM = 1500.0   # 相机距轨面距离

# ═══ 上位机 ═══
UPPER_COMPUTER_URL = "http://192.168.1.100:5000"
UPLOAD_INTERVAL = 1.0          # 上传间隔 (秒)

# ═══ 数据库 ═══
LOCAL_DB_PATH = os.path.join(os.path.dirname(__file__), "nuc_local.db")

# ═══ ROS2 (可选) ═══
USE_ROS2 = False               # 设为 True 启用 ROS2 话题
ROS_NODE_NAME = "rail_inspect_nuc"
