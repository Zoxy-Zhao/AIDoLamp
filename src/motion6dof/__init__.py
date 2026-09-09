"""Hardware-independent six-axis lamp motion algorithms (metres, radians)."""

from .kinematics import ArmGeometry, IKError, Pose, SixAxisArm
from .trajectory import JointTrajectory, interpolate_joints
from .tracking import LampPlanner, TrackingPlan

__all__ = [
    "ArmGeometry", "IKError", "Pose", "SixAxisArm", "JointTrajectory",
    "interpolate_joints", "LampPlanner", "TrackingPlan",
]
