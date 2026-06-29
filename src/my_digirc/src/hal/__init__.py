"""
Hardware Abstraction Layer (HAL) for drone control.

Re-exports the abstract base class and implementations.
"""

from hal.abstract_hal import AbstractDroneHAL
from hal.sitl_hal import SitlHAL

__all__ = ["AbstractDroneHAL", "SitlHAL"]
