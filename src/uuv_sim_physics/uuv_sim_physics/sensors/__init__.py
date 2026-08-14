"""Sensor observation models.

Everything here takes measurements in and produces observations out. No module
in this package may consume privileged simulator state -- they are listed in
``privileged.OBSERVATION_PRODUCERS`` and the isolation test enforces it.
"""

from .optical import WaterColumn, contrast_metrics, degrade

__all__ = ["WaterColumn", "degrade", "contrast_metrics"]
