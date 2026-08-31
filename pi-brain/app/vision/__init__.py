"""树莓派视觉组件。

本包只负责摄像头采集、OpenCV 检测和本地观察结果发布；不得导入串口桥、
Navigator、RouteRunner 或 STM32 协议。运动决策必须由任务层消费观察结果后完成。
"""

from .models import (
    CameraFrame,
    LineObservation,
    MaterialObservation,
    ObstacleObservation,
    QrObservation,
    RoadObservation,
    VisionHealth,
    VisionObservation,
)
from .service import VisionService
from .factory import build_vision_pipeline

__all__ = [
    "CameraFrame",
    "LineObservation",
    "MaterialObservation",
    "ObstacleObservation",
    "QrObservation",
    "RoadObservation",
    "VisionHealth",
    "VisionObservation",
    "VisionService",
    "build_vision_pipeline",
]
