"""Analytic IK for an ideal 3R positioning arm with a Z-Y-Z spherical wrist.

This is a new software model, not a model calibrated to the four-servo lamp.
R03 = Rz(q1) Ry(q2 + q3); R36 = Rz(q4) Ry(q5) Rz(q6).
The lamp optical axis is local +Z, with the TCP offset along that axis.
"""

from dataclasses import dataclass, field
from itertools import product
import math

import numpy as np


class IKError(ValueError):
    """No joint-limit-compliant solution exists for the requested pose."""


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result.copy()


def rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


@dataclass
class Pose:
    position: np.ndarray
    rotation: np.ndarray

    def __post_init__(self):
        self.position = vector(self.position, 3, "position")
        self.rotation = np.asarray(self.rotation, dtype=float).copy()
        if (
            self.rotation.shape != (3, 3)
            or not np.all(np.isfinite(self.rotation))
            or not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-9, rtol=0)
            or not np.isclose(np.linalg.det(self.rotation), 1.0, atol=1e-9, rtol=0)
        ):
            raise ValueError("rotation must be a proper orthonormal 3x3 matrix")


@dataclass
class ArmGeometry:
    base_height: float = 0.08
    upper_arm: float = 0.24
    forearm: float = 0.24
    tool_offset: float = 0.08
    joint_limits: np.ndarray = field(default_factory=lambda: np.deg2rad([
        [-180, 180], [-170, 170], [-155, 155],
        [-180, 180], [-180, 180], [-180, 180],
    ]))

    def __post_init__(self):
        lengths = vector(
            [self.base_height, self.upper_arm, self.forearm, self.tool_offset], 4, "geometry"
        )
        if np.any(lengths < 0) or self.upper_arm <= 0 or self.forearm <= 0:
            raise ValueError("link lengths must be positive; base/tool offsets may be zero")
        self.joint_limits = np.asarray(self.joint_limits, dtype=float).copy()
        if (
            self.joint_limits.shape != (6, 2)
            or not np.all(np.isfinite(self.joint_limits))
            or np.any(self.joint_limits[:, 0] >= self.joint_limits[:, 1])
            or np.any(np.diff(self.joint_limits, axis=1) > 2 * np.pi + 1e-10)
        ):
            raise ValueError("provide six finite increasing limits, each spanning at most 2*pi")


class SixAxisArm:
    def __init__(self, geometry=None):
        self.geometry = geometry if geometry is not None else ArmGeometry()

    def check_joints(self, joints):
        q = vector(joints, 6, "joints")
        limits = self.geometry.joint_limits
        if np.any(q < limits[:, 0] - 1e-10) or np.any(q > limits[:, 1] + 1e-10):
            raise ValueError("joints exceed configured limits")
        return q

    def forward(self, joints):
        q = self.check_joints(joints)
        g = self.geometry
        shoulder = np.array([0.0, 0.0, g.base_height])
        r01 = rz(q[0])
        r03 = r01 @ ry(q[1] + q[2])
        wrist = shoulder + r01 @ ry(q[1]) @ np.array([g.upper_arm, 0, 0])
        wrist += r03 @ np.array([g.forearm, 0, 0])
        rotation = r03 @ rz(q[3]) @ ry(q[4]) @ rz(q[5])
        return Pose(wrist + g.tool_offset * rotation[:, 2], rotation)

    def joint_positions(self, joints):
        q = self.check_joints(joints)
        g = self.geometry
        shoulder = np.array([0.0, 0.0, g.base_height])
        elbow = shoulder + rz(q[0]) @ ry(q[1]) @ np.array([g.upper_arm, 0, 0])
        wrist = elbow + rz(q[0]) @ ry(q[1] + q[2]) @ np.array([g.forearm, 0, 0])
        # The three wrist rotation axes intersect at the same point.
        return np.array([np.zeros(3), shoulder, elbow, wrist, self.forward(q).position])

    def _fit(self, angle, joint, reference):
        lo, hi = self.geometry.joint_limits[joint]
        period = 2 * np.pi
        first = math.ceil((lo - angle - 1e-10) / period)
        last = math.floor((hi - angle + 1e-10) / period)
        values = [float(np.clip(angle + k * period, lo, hi)) for k in range(first, last + 1)]
        return min(values, key=lambda v: abs(v - reference)) if values else None

    def _wrist_solutions(self, rotation, seed):
        beta = np.arccos(np.clip(rotation[2, 2], -1, 1))
        if abs(np.sin(beta)) > 1e-8:
            alpha = np.arctan2(rotation[1, 2], rotation[0, 2])
            gamma = np.arctan2(rotation[2, 1], -rotation[2, 0])
            return [(alpha, beta, gamma), (alpha + np.pi, -beta, gamma + np.pi)]

        # At beta=0, alpha+gamma is constrained; at beta=pi, alpha-gamma is.
        at_zero = rotation[2, 2] > 0
        angle = (
            np.arctan2(rotation[1, 0], rotation[0, 0]) if at_zero
            else np.arctan2(-rotation[1, 0], -rotation[0, 0])
        )
        alpha_candidates = [seed[3], 0.0, *self.geometry.joint_limits[3]]
        for gamma in [seed[5], *self.geometry.joint_limits[5]]:
            alpha_candidates.append(angle - gamma if at_zero else angle + gamma)
        return [
            (alpha, 0.0 if at_zero else np.pi, angle - alpha if at_zero else alpha - angle)
            for alpha in alpha_candidates
        ]

    def inverse_all(self, pose, seed=None):
        """Enumerate base/elbow/wrist branches and verify every result by FK.

        Singular wrist families are represented by limit-compliant candidates
        near the seed, not by an infinite list. Base-axis singularities likewise
        use the seed's base angle (plus its opposite).
        """
        reference = self.check_joints(seed) if seed is not None else np.mean(
            self.geometry.joint_limits, axis=1
        )
        g = self.geometry
        wrist = pose.position - g.tool_offset * pose.rotation[:, 2]
        relative = wrist - np.array([0.0, 0.0, g.base_height])
        radial = np.hypot(relative[0], relative[1])
        c3 = (radial**2 + relative[2]**2 - g.upper_arm**2 - g.forearm**2) / (
            2 * g.upper_arm * g.forearm
        )
        if c3 < -1 - 1e-10 or c3 > 1 + 1e-10:
            return []
        base = np.arctan2(relative[1], relative[0]) if radial > 1e-10 else reference[0]
        elbow = np.arccos(np.clip(c3, -1, 1))
        solutions = []
        for q1, q3 in product([base, base + np.pi], [elbow, -elbow]):
            signed_r = relative[0] * np.cos(q1) + relative[1] * np.sin(q1)
            q2 = np.arctan2(-relative[2], signed_r) - np.arctan2(
                g.forearm * np.sin(q3), g.upper_arm + g.forearm * np.cos(q3)
            )
            r03 = rz(q1) @ ry(q2 + q3)
            for wrist_q in self._wrist_solutions(r03.T @ pose.rotation, reference):
                candidate = [
                    self._fit(value, index, reference[index])
                    for index, value in enumerate((q1, q2, q3, *wrist_q))
                ]
                if any(value is None for value in candidate):
                    continue
                q = np.array(candidate)
                actual = self.forward(q)
                if (
                    np.linalg.norm(actual.position - pose.position) > 1e-7
                    or np.linalg.norm(actual.rotation - pose.rotation) > 1e-7
                ):
                    continue
                if not any(np.allclose(q, old, atol=1e-8, rtol=0) for old in solutions):
                    solutions.append(q)
        return sorted(solutions, key=lambda q: float(np.linalg.norm(q - reference)))

    def inverse(self, pose, seed=None):
        solutions = self.inverse_all(pose, seed)
        if not solutions:
            raise IKError("pose has no verified IK solution within configured limits")
        return solutions[0]

    def project_workspace(self, pose):
        """Project the wrist into the link-length shell, preserving orientation.

        This geometric projection does not resolve joint limits or collisions;
        callers must still run IK and handle rejection.
        """
        g = self.geometry
        shoulder = np.array([0.0, 0.0, g.base_height])
        wrist = pose.position - g.tool_offset * pose.rotation[:, 2]
        delta = wrist - shoulder
        distance = np.linalg.norm(delta)
        inner = abs(g.upper_arm - g.forearm) + 1e-6
        outer = g.upper_arm + g.forearm - 1e-6
        if inner > outer:
            raise ValueError("link lengths are too small for workspace projection tolerance")
        projected_distance = np.clip(distance, inner, outer)
        direction = delta / distance if distance > 1e-12 else np.array([1.0, 0, 0])
        position = shoulder + direction * projected_distance + g.tool_offset * pose.rotation[:, 2]
        return Pose(position, pose.rotation), bool(abs(projected_distance - distance) > 1e-12)
