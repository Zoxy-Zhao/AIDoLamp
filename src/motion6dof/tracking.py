"""Task-space lamp aiming; inputs are targets already expressed in base frame."""

from dataclasses import dataclass

import numpy as np

from .kinematics import IKError, Pose, SixAxisArm, rz, vector
from .trajectory import JointTrajectory, interpolate_joints


@dataclass
class TrackingPlan:
    mode: str
    target: np.ndarray
    pose: Pose
    joints: np.ndarray
    trajectory: JointTrajectory
    workspace_projected: bool


def look_at(position, target, roll=0.0):
    position = vector(position, 3, "position")
    target = vector(target, 3, "target")
    if not np.isfinite(roll):
        raise ValueError("roll must be finite")
    optical_axis = target - position
    distance = np.linalg.norm(optical_axis)
    if distance < 1e-9:
        raise ValueError("lamp and target must be distinct")
    optical_axis /= distance
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(up, optical_axis)) > 0.95:
        up = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(up, optical_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(optical_axis, x_axis)
    return np.column_stack([x_axis, y_axis, optical_axis]) @ rz(roll)


class LampPlanner:
    def __init__(self, arm=None):
        self.arm = arm if arm is not None else SixAxisArm()

    def plan(self, target, current, mode="book", distance=0.30, duration=2.0,
             steps=100, max_velocity=0.8, allow_projection=False):
        """Plan one rest-to-rest move; book distance is a TCP-to-target offset.

        Face mode holds the first three joints, so its distance is determined
        by the current wrist position rather than the book distance argument.
        """
        target = vector(target, 3, "target")
        current = self.arm.check_joints(current)
        if mode not in ("face", "book"):
            raise ValueError("mode must be 'face' or 'book'")
        if not np.isfinite(distance) or distance <= 0:
            raise ValueError("distance must be positive and finite")
        projected = False
        if mode == "face":
            # Keep positioning joints fixed; aim with only the spherical wrist.
            # For a +Z tool offset, aim from the wrist centre to the target.
            old = self.arm.forward(current)
            wrist = old.position - self.arm.geometry.tool_offset * old.rotation[:, 2]
            if np.linalg.norm(target - wrist) <= self.arm.geometry.tool_offset + 1e-9:
                raise IKError("face target must lie beyond the lamp tool offset")
            rotation = look_at(wrist, target)
            pose = Pose(wrist + self.arm.geometry.tool_offset * rotation[:, 2], rotation)
            candidates = self.arm.inverse_all(pose, seed=current)
            candidates = [q for q in candidates if np.allclose(q[:3], current[:3], atol=1e-7, rtol=0)]
            if not candidates:
                raise IKError("target cannot be aimed at with positioning joints fixed")
            goal = candidates[0]
        else:
            # Book mode offsets the lamp upward and coordinates all six joints.
            position = target + np.array([0.0, 0.0, distance])
            pose = Pose(position, look_at(position, target))
            if allow_projection:
                # Projection preserves orientation and may shift the optical ray
                # away from the target. Re-aim and require verified IK afterwards.
                pose, projected = self.arm.project_workspace(pose)
                if projected:
                    pose = Pose(pose.position, look_at(pose.position, target))
            goal = self.arm.inverse(pose, seed=current)
        trajectory = interpolate_joints(
            self.arm, current, goal, duration, steps, max_velocity
        )
        return TrackingPlan(mode, target, pose, goal, trajectory, projected)
