"""Mode-aware adaptive navigation for UUVs under multi-modal sensing degradation.

Companion software for the study *Mode-Aware Adaptive Navigation for UUVs Using
Multi-Modal Sensing and Optical Feedback in Simulation Environment*.

The package is deliberately split so that the statistical campaign never depends
on ROS or on a renderer:

``optics``
    Underwater optical propagation and channel availability. Pure NumPy.

Modules added later (mode manager, comparators, campaign runner) follow the same
rule: physics and decision logic stay importable without a ROS environment, and
ROS nodes are thin wrappers used only for the qualitative demonstration.
"""

__version__ = "0.1.0"
