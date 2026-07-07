"""
YOLOv8 rail defect detector
Intel NUC / Jetson Xavier · PyTorch + OpenCV
"""

import time
import math
import cv2
import numpy as np
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

from config import (
    YOLO_MODEL_PATH, YOLO_CONF, YOLO_IOU, YOLO_IMG_SIZE,
    DETECT_CLASSES, CAMERA_MATRIX, WORKING_DISTANCE_MM,
)


@dataclass
class DefectResult:
    label: str
    class_id: int
    confidence: float
    bbox: Tuple[int, int, int, int]   # x, y, w, h (pixels)
    size_mm: Tuple[float, float, float]  # L, W, D
    alert_level: int                    # 1=mild, 2=moderate, 3=severe
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "label": self.label,
            "class_id": self.class_id,
            "confidence": round(self.confidence, 3),
            "bbox": list(self.bbox),
            "size_mm": [round(v, 1) for v in self.size_mm],
            "alert_level": self.alert_level,
            "timestamp": self.timestamp,
        }


class DefectDetector:
    """Rail surface defect detection using YOLOv8"""

    # Alert thresholds per class (L×W×D, mm)
    THRESHOLDS = {
        0: (5, 20),    # crack
        1: (10, 50),   # spall
        2: (15, 60),   # squat
        3: (0.3, 1.0), # corrugation (depth key)
        4: (1, 3),     # loosened fastener
        5: (5, 15),    # rail burn
    }

    def __init__(self, model_path: str = YOLO_MODEL_PATH):
        self.model = None
        self.model_path = Path(model_path)
        self._load_model()
        self.fx = CAMERA_MATRIX[0][0]
        self.fy = CAMERA_MATRIX[1][1]
        self.work_dist = WORKING_DISTANCE_MM
        self._detect_count = 0

    def _load_model(self):
        if self.model_path.suffix == ".pt":
            try:
                from ultralytics import YOLO
                self.model = YOLO(str(self.model_path))
                print(f"[Detector] YOLOv8 model loaded: {self.model_path}")
            except ImportError:
                print("[Detector] ultralytics not installed, using mock")
                self.model = None
        else:
            print("[Detector] model file not found, using mock")
            self.model = None

    def preprocess(self, img: np.ndarray) -> np.ndarray:
        """CLAHE + noise reduction"""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        return cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)

    def detect(self, image: np.ndarray,
               thermal_max_temp: float = 28.0) -> List[DefectResult]:
        """Full detection pipeline"""

        preprocessed = self.preprocess(image)
        raw = self._inference(preprocessed)
        return self._postprocess(raw, thermal_max_temp)

    def _inference(self, img: np.ndarray) -> List[dict]:
        if self.model is not None:
            results = self.model(img, conf=YOLO_CONF, iou=YOLO_IOU,
                                 imgsz=YOLO_IMG_SIZE, verbose=False)
            detections = []
            for r in results:
                if r.boxes is None:
                    continue
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    detections.append({
                        "class": int(box.cls[0]),
                        "conf": float(box.conf[0]),
                        "bbox": (int(x1), int(y1),
                                 int(x2 - x1), int(y2 - y1)),
                    })
            return detections
        else:
            # Mock for testing without GPU
            return self._mock_inference(img)

    def _mock_inference(self, img: np.ndarray) -> List[dict]:
        """Simulated detections for offline testing"""
        import random
        if random.random() < 0.3:
            cls_id = random.randint(0, 4)
            cw, ch = img.shape[1], img.shape[0]
            x = random.randint(50, cw - 100)
            y = random.randint(50, ch - 100)
            w = random.randint(30, 200)
            h = random.randint(20, 100)
            return [{
                "class": cls_id,
                "conf": random.uniform(0.55, 0.98),
                "bbox": (x, y, w, h),
            }]
        return []

    def _postprocess(self, raw: List[dict],
                     thermal_max_temp: float) -> List[DefectResult]:
        results = []
        for det in raw:
            cls_id = det["class"]
            conf = det["conf"]
            bbox = det["bbox"]
            size = self._estimate_size(bbox, cls_id, thermal_max_temp)
            alert = self._classify_alert(cls_id, size)

            results.append(DefectResult(
                label=DETECT_CLASSES.get(cls_id, f"class_{cls_id}"),
                class_id=cls_id,
                confidence=conf,
                bbox=bbox,
                size_mm=size,
                alert_level=alert,
            ))

        self._detect_count += len(results)
        return results

    def _estimate_size(self, bbox: Tuple[int, int, int, int],
                       cls_id: int, thermal_hint: float
                       ) -> Tuple[float, float, float]:
        """Pixel to physical size using camera intrinsics"""
        _, _, bw, bh = bbox
        scale_x = self.work_dist / self.fx
        scale_y = self.work_dist / self.fy
        length = bw * scale_x
        width = bh * scale_y
        depth = 0.0

        if cls_id == 1:  # spall: IR temp gap hints depth
            depth = max(0, (thermal_hint - 28.0) * 0.8)
        elif cls_id == 3:  # corrugation: tiny depth
            depth = max(0, thermal_hint * 0.05)
        else:
            depth = 0.0

        return (round(length, 1), round(width, 1), round(depth, 1))

    def _classify_alert(self, cls_id: int,
                        size: Tuple[float, float, float]) -> int:
        l, w, d = size
        thr = self.THRESHOLDS.get(cls_id, (10, 50))
        l1, l2 = thr

        if cls_id == 3:   # corrugation: depth matters
            worst = d
        elif cls_id == 1: # spall: all dimensions
            worst = max(l, w, d)
        else:
            worst = max(l, w)

        if worst < l1:
            return 1
        elif worst < l2:
            return 2
        else:
            return 3


class ThermalReader:
    """MLX90640 / AMG8833 IR thermal array reader"""

    def __init__(self):
        self._temp = 28.0

    def read_max_temp(self) -> float:
        """Read maximum temperature in frame"""
        # Real: I2C read from MLX90640, find max
        return self._temp

    def set_mock_temp(self, t: float):
        self._temp = t
