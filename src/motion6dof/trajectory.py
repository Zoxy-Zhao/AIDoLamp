"""Quintic joint-space interpolation with explicit timestamps and derivatives."""

from dataclasses import dataclass

import numpy as np

from .kinematics import vector


@dataclass
class JointTrajectory:
    time: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


def interpolate_joints(arm, start, end, duration=2.0, steps=100, max_velocity=None):
    start = arm.check_joints(start)
    end = arm.check_joints(end)
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be positive and finite")
    if isinstance(steps, bool) or not isinstance(steps, (int, np.integer)) or steps < 2:
        raise ValueError("steps must be an integer >= 2")
    delta = end - start
    if max_velocity is not None:
        limits = np.asarray(max_velocity, dtype=float)
        if limits.ndim == 0:
            limits = np.full(6, float(limits))
        limits = vector(limits, 6, "max_velocity")
        if np.any(limits <= 0):
            raise ValueError("max_velocity must be positive")
        # Maximum derivative of 10u^3-15u^4+6u^5 is 1.875.
        duration = max(duration, float(np.max(1.875 * np.abs(delta) / limits)))
    u = np.linspace(0.0, 1.0, steps + 1)
    blend = 10 * u**3 - 15 * u**4 + 6 * u**5
    first = (30 * u**2 - 60 * u**3 + 30 * u**4) / duration
    second = (60 * u - 180 * u**2 + 120 * u**3) / duration**2
    return JointTrajectory(
        time=u * duration,
        position=start + blend[:, None] * delta,
        velocity=first[:, None] * delta,
        acceleration=second[:, None] * delta,
    )
