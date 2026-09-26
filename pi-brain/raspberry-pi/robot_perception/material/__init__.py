"""物料颜色、位置、形状和姿态识别。"""

from .detector import (
    GripperDetection,
    GripperMaterialDetector,
    MaterialObservation,
)

__all__ = [
    "GripperDetection",
    "GripperMaterialDetector",
    "MaterialObservation",
]
