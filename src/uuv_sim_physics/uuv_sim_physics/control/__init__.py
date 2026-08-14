"""Guidance, control and thrust allocation for the physics backend.

Import-safe without Gazebo: allocation and control are pure NumPy. Only
``runner`` reaches the plant, and it imports ``gazebo_backend`` itself.
"""

from .allocation import Allocator, Wrench, JOINT_ORDER
from .controller import Controller, Gains, Reference, State, MODE

__all__ = ["Allocator", "Wrench", "JOINT_ORDER",
           "Controller", "Gains", "Reference", "State", "MODE"]
