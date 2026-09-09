"""Numerical tests of the new six-axis software model; no hardware fixtures."""

import unittest

import numpy as np

from src.motion6dof import ArmGeometry, IKError, LampPlanner, Pose, SixAxisArm, interpolate_joints
from src.motion6dof.kinematics import rz
from src.motion6dof.tracking import look_at


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.arm = SixAxisArm()

    def test_known_forward_pose(self):
        pose = self.arm.forward(np.zeros(6))
        np.testing.assert_allclose(pose.position, [0.48, 0, 0.16], atol=1e-12)
        np.testing.assert_allclose(pose.rotation, np.eye(3), atol=1e-12)
        pose = self.arm.forward([np.pi / 2, -np.pi / 2, 0, 0, np.pi / 2, 0])
        np.testing.assert_allclose(pose.position, [0, 0, 0.64], atol=1e-12)
        np.testing.assert_allclose(pose.rotation, rz(np.pi / 2), atol=1e-12)

    def test_random_fk_ik_roundtrips(self):
        random = np.random.default_rng(20260909)
        limits = self.arm.geometry.joint_limits
        for _ in range(250):
            q = random.uniform(limits[:, 0] + 0.02, limits[:, 1] - 0.02)
            pose = self.arm.forward(q)
            solutions = self.arm.inverse_all(pose)
            self.assertTrue(solutions)
            for solution in solutions:
                actual = self.arm.forward(solution)
                np.testing.assert_allclose(actual.position, pose.position, atol=1e-7)
                np.testing.assert_allclose(actual.rotation, pose.rotation, atol=1e-7)
            recovered = self.arm.inverse(pose, seed=q)
            np.testing.assert_allclose(recovered, q, atol=1e-7)

    def test_wrist_singularities(self):
        for q5 in [0, np.pi, -np.pi, 1e-9]:
            q = np.array([0.4, -0.5, 0.9, 0.6, q5, -0.3])
            expected = self.arm.forward(q)
            actual = self.arm.forward(self.arm.inverse(expected, seed=q))
            np.testing.assert_allclose(actual.position, expected.position, atol=1e-7)
            np.testing.assert_allclose(actual.rotation, expected.rotation, atol=1e-7)

    def test_singular_wrist_with_asymmetric_limits(self):
        limits = self.arm.geometry.joint_limits.copy()
        limits[3] = [0.5, 1.0]
        limits[5] = [0.8, 1.2]
        restricted = SixAxisArm(ArmGeometry(joint_limits=limits))
        q = [0.4, -0.5, 0.9, 0.8, 0, 1.0]
        pose = restricted.forward(q)
        actual = restricted.forward(restricted.inverse(pose))
        np.testing.assert_allclose(actual.rotation, pose.rotation, atol=1e-7)

    def test_base_axis_and_full_extension(self):
        for q in [[0.7, -np.pi / 2, 0, 0.2, 0.6, 0.4], [0, 0, 0, 0, 0, 0]]:
            pose = self.arm.forward(q)
            actual = self.arm.forward(self.arm.inverse(pose, seed=q))
            np.testing.assert_allclose(actual.position, pose.position, atol=1e-7)
            np.testing.assert_allclose(actual.rotation, pose.rotation, atol=1e-7)

    def test_unreachable_and_projection(self):
        distant = Pose([2, 0, 0.16], np.eye(3))
        with self.assertRaises(IKError):
            self.arm.inverse(distant)
        corrected, changed = self.arm.project_workspace(distant)
        self.assertTrue(changed)
        actual = self.arm.forward(self.arm.inverse(corrected))
        np.testing.assert_allclose(actual.position, corrected.position, atol=1e-7)

    def test_joint_limits_reject_pose_without_clipping(self):
        restricted = SixAxisArm(ArmGeometry(joint_limits=np.tile([-0.05, 0.05], (6, 1))))
        pose = self.arm.forward([1, -0.4, 0.8, 0.3, 0.6, 0.2])
        with self.assertRaises(IKError):
            restricted.inverse(pose)

    def test_seed_selects_elbow_branch(self):
        q = np.array([0.2, -0.6, 1.0, 0.4, 0.8, 0.2])
        pose = self.arm.forward(q)
        solutions = self.arm.inverse_all(pose, q)
        self.assertGreaterEqual(len(solutions), 4)
        for candidate in solutions:
            np.testing.assert_allclose(self.arm.inverse(pose, candidate), candidate, atol=1e-7)

    def test_invalid_inputs(self):
        for position, rotation in [([np.nan, 0, 0], np.eye(3)), ([0, 0, 0], np.ones((3, 3))),
                                   ([0, 0, 0], np.diag([1, 1, -1]))]:
            with self.assertRaises(ValueError):
                Pose(position, rotation)
        with self.assertRaises(ValueError):
            self.arm.forward([0, 0, 0, 0])
        with self.assertRaises(ValueError):
            ArmGeometry(upper_arm=-1)


class TrajectoryTests(unittest.TestCase):
    def test_endpoints_limits_and_speed(self):
        arm = SixAxisArm()
        start = np.array([-2.9, -1, 1, 0, 0.4, -0.5])
        end = np.array([2.9, 0.2, -0.7, 1, -0.4, 0.5])
        trajectory = interpolate_joints(arm, start, end, duration=0.1, steps=200, max_velocity=0.5)
        np.testing.assert_allclose(trajectory.position[[0, -1]], [start, end])
        np.testing.assert_allclose(trajectory.velocity[[0, -1]], 0, atol=1e-12)
        np.testing.assert_allclose(trajectory.acceleration[[0, -1]], 0, atol=1e-12)
        self.assertLessEqual(np.max(np.abs(trajectory.velocity)), 0.5 + 1e-12)
        self.assertTrue(np.all(np.diff(trajectory.time) > 0))
        for q in trajectory.position:
            arm.check_joints(q)
        # Check the supplied analytical velocity against numerical derivatives.
        numerical = np.gradient(trajectory.position, trajectory.time, axis=0)
        np.testing.assert_allclose(numerical[2:-2], trajectory.velocity[2:-2], atol=1e-4)

    def test_stationary_and_invalid_requests(self):
        arm = SixAxisArm()
        trajectory = interpolate_joints(arm, np.zeros(6), np.zeros(6))
        np.testing.assert_allclose(trajectory.position, 0)
        for kwargs in [{"steps": 0}, {"steps": 2.5}, {"duration": -1}, {"max_velocity": 0},
                       {"max_velocity": float("nan")}, {"duration": float("inf")}]:
            with self.assertRaises(ValueError):
                interpolate_joints(arm, np.zeros(6), np.zeros(6), **kwargs)


class TrackingTests(unittest.TestCase):
    def test_book_then_face_aiming(self):
        planner = LampPlanner()
        current = np.deg2rad([0, -45, 90, 0, 45, 0])
        book = planner.plan([0.24, 0.06, 0], current, "book")
        self.assertFalse(book.workspace_projected)
        np.testing.assert_allclose(np.linalg.norm(book.pose.position - book.target), 0.30)
        face = planner.plan([0.50, 0.10, 0.38], book.joints, "face")
        np.testing.assert_allclose(face.joints[:3], book.joints[:3], atol=1e-7)
        for plan in [book, face]:
            actual = planner.arm.forward(plan.joints)
            direction = plan.target - actual.position
            direction /= np.linalg.norm(direction)
            np.testing.assert_allclose(actual.rotation[:, 2], direction, atol=1e-7)

    def test_nearby_targets_and_invalid_mode(self):
        planner = LampPlanner()
        first = planner.plan([0.24, 0.06, 0], np.deg2rad([0, -45, 90, 0, 45, 0]))
        second = planner.plan([0.241, 0.061, 0], first.joints)
        self.assertLess(np.max(np.abs(second.joints - first.joints)), 0.1)
        with self.assertRaises(ValueError):
            planner.plan([0.2, 0, 0], first.joints, "unknown")
        with self.assertRaises(ValueError):
            look_at([0, 0, 0], [0, 0, 0])

    def test_face_target_inside_tool_offset_is_rejected(self):
        planner = LampPlanner()
        q = np.zeros(6)
        pose = planner.arm.forward(q)
        wrist = pose.position - planner.arm.geometry.tool_offset * pose.rotation[:, 2]
        with self.assertRaises(IKError):
            planner.plan(wrist + [0, 0, 0.01], q, "face")


if __name__ == "__main__":
    unittest.main()
