"""Run a synthetic target demo without cameras, serial ports or model weights."""

import argparse
import json
from pathlib import Path

import numpy as np

from .kinematics import SixAxisArm
from .tracking import LampPlanner


def plot_demo(arm, reports, path):
    """Optional plot of calculated poses and time-parameterized joint commands."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(13, 6), facecolor="#f7f9fc")
    spatial = fig.add_subplot(121, projection="3d")
    joints = fig.add_subplot(122)
    colors = ["#2157a5", "#469bbc", "#b76c20", "#d99b3c"]
    for index, report in enumerate(reports):
        q = np.deg2rad(report["joints_deg"])
        points = arm.joint_positions(q)
        target = np.array(report["target_m"])
        spatial.plot(*points.T, "o-", color=colors[index], label=f'{index + 1}: {report["mode"]}')
        spatial.plot(*np.array([points[-1], target]).T, "--", color=colors[index], alpha=0.6)
        spatial.scatter(*target, marker="*", s=90, color=colors[index])
    spatial.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)", title="Calculated arm poses and optical rays")
    spatial.set_box_aspect((1, 0.7, 1))
    spatial.legend(loc="upper left", fontsize=8)
    offset = 0.0
    for report in reports:
        trajectory = report["trajectory"]
        times = np.array(trajectory["time_s"]) + offset
        angles = np.rad2deg(trajectory["position_rad"])
        for index in range(6):
            joints.plot(times, angles[:, index], color=f"C{index}", label=f"J{index + 1}" if offset == 0 else None)
        if offset:
            joints.axvline(offset, color="#cbd5e1", linestyle=":")
        offset = float(times[-1])
    joints.set(xlabel="Planned time (s)", ylabel="Joint command (degrees)", title="Six joint trajectories | speed limit 0.8 rad/s")
    joints.grid(alpha=0.2)
    joints.legend(ncol=3, fontsize=9)
    fig.suptitle("AIDoLamp | Six-axis motion algorithm demo", fontsize=17, fontweight="bold")
    fig.text(0.5, 0.02, "Synthetic targets / ideal spherical wrist / no hardware execution", ha="center", color="#64748b")
    fig.tight_layout(rect=(0, 0.05, 1, 0.93))
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Optional JSON output path")
    parser.add_argument("--plot", type=Path, help="Optional plot path (requires matplotlib)")
    args = parser.parse_args()
    arm = SixAxisArm()
    planner = LampPlanner(arm)
    current = np.deg2rad([0, -45, 90, 0, 45, 0])
    reports = []
    for mode, target in [
        ("book", [0.24, 0.06, 0.0]),
        ("book", [0.25, 0.07, 0.0]),
        ("face", [0.50, 0.10, 0.38]),
        ("face", [0.51, 0.12, 0.39]),
    ]:
        plan = planner.plan(target, current, mode=mode)
        actual = arm.forward(plan.joints)
        reports.append({
            "mode": mode,
            "target_m": plan.target.tolist(),
            "tcp_m": actual.position.tolist(),
            "joints_deg": np.rad2deg(plan.joints).tolist(),
            "fk_position_residual_m": float(np.linalg.norm(actual.position - plan.pose.position)),
            "fk_rotation_matrix_residual": float(np.linalg.norm(actual.rotation - plan.pose.rotation)),
            "duration_s": float(plan.trajectory.time[-1]),
            "workspace_projected": plan.workspace_projected,
            "trajectory": {
                "time_s": plan.trajectory.time.tolist(),
                "position_rad": plan.trajectory.position.tolist(),
                "velocity_rad_s": plan.trajectory.velocity.tolist(),
                "acceleration_rad_s2": plan.trajectory.acceleration.tolist(),
            },
        })
        current = plan.joints
    result = {
        "kind": "synthetic_six_axis_algorithm_demo",
        "hardware_validated": False,
        "description": "New ideal spherical-wrist model; no camera or STM32 execution.",
        "plans": reports,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    if args.plot:
        plot_demo(arm, reports, args.plot)
    summary = {**result, "plans": [{k: v for k, v in r.items() if k != "trajectory"} for r in reports]}
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
