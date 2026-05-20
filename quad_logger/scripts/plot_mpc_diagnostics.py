"""MPC diagnostic plots from a quad-sdk hardware MCAP bag.

Generates 5 figures per bag:
  1. Commanded vs actual ground reaction force (GRF) per leg.
  2. Commanded vs actual joint torque per joint, per leg.
  3. Per-leg foot velocity (x/y/z + speed magnitude).
  4. Body orientation (RPY) and angular velocity.
  5. Planned vs actual contact timing per leg.

Sources (quad-sdk leg order: leg_0=FL, leg_1=RL, leg_2=FR, leg_3=RR):
  GRF cmd          /robot_1/control/grfs                  GRFArray.vectors[i]
  GRF actual (est) /robot_1/body_force/toe_forces         GRFArray.vectors[i]
  tau cmd          /robot_1/control/joint_command         LegCommandArray.leg_commands[leg].motor_commands[joint].effort
  tau actual (est) /robot_1/body_force/joint_torques      BodyForceEstimate.joint_torques[3*leg+joint]
  foot vel         /robot_1/state/ground_truth            RobotState.feet.feet[i].velocity
  body RPY/omega   /robot_1/state/ground_truth            body.pose.orientation, body.twist.angular
  contact planned  /robot_1/local_plan                    RobotPlan.grfs[0].contact_states[i]
  contact actual   /robot_1/state/foot_contact            FootContact.contact_states[i] (+ foot_force_raw[i] overlay)
"""

import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R


LEG_NAMES = ["FL", "RL", "FR", "RR"]
JOINT_NAMES = ["Abd", "Hip", "Knee"]


def stamp_seconds(header) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def quat_to_rpy(q):
    return R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")


def read_topics(bag_path: str, topics: set[str]) -> dict[str, list]:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id="mcap"),
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out: dict[str, list] = {t: [] for t in topics}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic not in topics:
            continue
        msg = deserialize_message(raw, get_message(type_map[topic]))
        out[topic].append(msg)
    return out


def parse_bag(bag_path: str) -> dict:
    topics = {
        "/robot_1/control/grfs",
        "/robot_1/body_force/toe_forces",
        "/robot_1/control/joint_command",
        "/robot_1/body_force/joint_torques",
        "/robot_1/state/ground_truth",
        "/robot_1/state/foot_contact",
        "/robot_1/local_plan",
    }
    msgs = read_topics(bag_path, topics)

    # Choose a single t0 (earliest header stamp across topics that have one).
    t0_candidates = []
    for topic in [
        "/robot_1/state/ground_truth",
        "/robot_1/control/grfs",
        "/robot_1/body_force/toe_forces",
        "/robot_1/control/joint_command",
        "/robot_1/body_force/joint_torques",
        "/robot_1/state/foot_contact",
    ]:
        if msgs[topic]:
            t0_candidates.append(stamp_seconds(msgs[topic][0].header))
    if msgs["/robot_1/local_plan"]:
        lp = msgs["/robot_1/local_plan"][0]
        t0_candidates.append(
            float(lp.state_timestamp.sec)
            + float(lp.state_timestamp.nanosec) * 1e-9)
    t0 = min(t0_candidates) if t0_candidates else 0.0

    data = {"t0": t0}

    # ------------------------------------------------------------ GRFs
    grf_cmd = msgs["/robot_1/control/grfs"]
    if grf_cmd:
        data["grf_cmd_t"] = np.array(
            [stamp_seconds(m.header) - t0 for m in grf_cmd])
        data["grf_cmd_xyz"] = np.array([
            [[m.vectors[i].x, m.vectors[i].y, m.vectors[i].z]
             for i in range(4)]
            for m in grf_cmd])  # (N, 4, 3)

    grf_act = msgs["/robot_1/body_force/toe_forces"]
    if grf_act:
        data["grf_act_t"] = np.array(
            [stamp_seconds(m.header) - t0 for m in grf_act])
        data["grf_act_xyz"] = np.array([
            [[m.vectors[i].x, m.vectors[i].y, m.vectors[i].z]
             for i in range(4)]
            for m in grf_act])  # (N, 4, 3)

    # ---------------------------------------------------------- Torques
    cmd = msgs["/robot_1/control/joint_command"]
    if cmd:
        data["tau_cmd_t"] = np.array(
            [stamp_seconds(m.header) - t0 for m in cmd])
        data["tau_cmd"] = np.array([
            [[m.leg_commands[leg].motor_commands[j].effort
              for j in range(3)]
             for leg in range(4)]
            for m in cmd])  # (N, 4, 3)
        data["tau_cmd_ff"] = np.array([
            [[m.leg_commands[leg].motor_commands[j].torque_ff
              for j in range(3)]
             for leg in range(4)]
            for m in cmd])

    tau_act = msgs["/robot_1/body_force/joint_torques"]
    if tau_act:
        data["tau_act_t"] = np.array(
            [stamp_seconds(m.header) - t0 for m in tau_act])
        # joint_torques is flat [12]; assume FL,RL,FR,RR x [Abd,Hip,Knee]
        data["tau_act"] = np.array([
            np.array(m.joint_torques, dtype=float).reshape(4, 3)
            for m in tau_act])  # (N, 4, 3)

    # ---------------------------------------------------- Body / feet
    gt = msgs["/robot_1/state/ground_truth"]
    if gt:
        data["gt_t"] = np.array([stamp_seconds(m.header) - t0 for m in gt])
        data["gt_rpy"] = np.array(
            [quat_to_rpy(m.body.pose.orientation) for m in gt])
        data["gt_omega"] = np.array([
            [m.body.twist.angular.x,
             m.body.twist.angular.y,
             m.body.twist.angular.z] for m in gt])
        data["gt_foot_vel"] = np.array([
            [[m.feet.feet[i].velocity.x,
              m.feet.feet[i].velocity.y,
              m.feet.feet[i].velocity.z]
             for i in range(len(m.feet.feet))]
            for m in gt])  # (N, 4, 3)

    # ----------------------------------------------------- Contact
    fc = msgs["/robot_1/state/foot_contact"]
    if fc:
        data["fc_t"] = np.array([stamp_seconds(m.header) - t0 for m in fc])
        data["fc_state"] = np.array(
            [list(m.contact_states) for m in fc], dtype=bool)
        data["fc_raw"] = np.array(
            [list(m.foot_force_raw) for m in fc], dtype=float)

    lp = msgs["/robot_1/local_plan"]
    if lp:
        data["lp_t"] = np.array([
            float(m.state_timestamp.sec)
            + float(m.state_timestamp.nanosec) * 1e-9 - t0
            for m in lp])
        data["lp_contact"] = np.array([
            list(m.grfs[0].contact_states) if m.grfs else [False]*4
            for m in lp], dtype=bool)

    return data


# ============================================================ plotting

def _legend_once(ax, *args, **kwargs):
    if not ax.get_legend():
        ax.legend(*args, **kwargs)


def plot_grfs(d: dict, title_prefix: str, save_dir: Path):
    if "grf_cmd_t" not in d or "grf_act_t" not in d:
        print("  skip GRF plot: missing data")
        return
    fig, axes = plt.subplots(4, 3, figsize=(15, 11), sharex=True)
    components = ["x", "y", "z"]
    for leg in range(4):
        for c in range(3):
            ax = axes[leg, c]
            ax.plot(d["grf_cmd_t"], d["grf_cmd_xyz"][:, leg, c],
                    color="C0", lw=1.2, label="commanded")
            ax.plot(d["grf_act_t"], d["grf_act_xyz"][:, leg, c],
                    color="C3", lw=0.8, alpha=0.8, label="actual (est.)")
            ax.grid(True, alpha=0.3)
            if c == 0:
                ax.set_ylabel(f"{LEG_NAMES[leg]}\nF [N]")
            if leg == 0:
                ax.set_title(f"GRF {components[c]}")
            if leg == 3:
                ax.set_xlabel("t [s]")
            if leg == 0 and c == 2:
                _legend_once(ax, loc="upper right", fontsize=9)
    fig.suptitle(f"{title_prefix} — GRF commanded vs actual (estimator)")
    fig.tight_layout()
    fig.savefig(save_dir / "01_grfs.png", dpi=130)
    plt.close(fig)


def plot_torques(d: dict, title_prefix: str, save_dir: Path):
    if "tau_cmd_t" not in d or "tau_act_t" not in d:
        print("  skip torque plot: missing data")
        return
    fig, axes = plt.subplots(4, 3, figsize=(15, 11), sharex=True)
    for leg in range(4):
        for j in range(3):
            ax = axes[leg, j]
            ax.plot(d["tau_cmd_t"], d["tau_cmd"][:, leg, j],
                    color="C0", lw=1.2, label="cmd (effort)")
            ax.plot(d["tau_cmd_t"], d["tau_cmd_ff"][:, leg, j],
                    color="C2", lw=0.8, alpha=0.6, label="cmd (torque_ff)")
            ax.plot(d["tau_act_t"], d["tau_act"][:, leg, j],
                    color="C3", lw=0.8, alpha=0.8, label="actual (est.)")
            ax.grid(True, alpha=0.3)
            if j == 0:
                ax.set_ylabel(f"{LEG_NAMES[leg]}\nτ [Nm]")
            if leg == 0:
                ax.set_title(f"{JOINT_NAMES[j]}")
            if leg == 3:
                ax.set_xlabel("t [s]")
            if leg == 0 and j == 2:
                _legend_once(ax, loc="upper right", fontsize=8)
    fig.suptitle(
        f"{title_prefix} — Joint torques: commanded effort, "
        "feedforward, and estimated actual")
    fig.tight_layout()
    fig.savefig(save_dir / "02_torques.png", dpi=130)
    plt.close(fig)


def plot_foot_velocity(d: dict, title_prefix: str, save_dir: Path):
    if "gt_t" not in d or "gt_foot_vel" not in d:
        print("  skip foot velocity plot: missing data")
        return
    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    t = d["gt_t"]
    v = d["gt_foot_vel"]  # (N,4,3)
    speed = np.linalg.norm(v, axis=2)  # (N,4)
    for leg in range(4):
        ax = axes[leg]
        ax.plot(t, v[:, leg, 0], color="C0", lw=0.8, label="vx")
        ax.plot(t, v[:, leg, 1], color="C2", lw=0.8, label="vy")
        ax.plot(t, v[:, leg, 2], color="C3", lw=0.8, label="vz")
        ax.plot(t, speed[:, leg], color="k", lw=1.0, alpha=0.6, label="|v|")
        ax.set_ylabel(f"{LEG_NAMES[leg]} [m/s]")
        ax.grid(True, alpha=0.3)
        if leg == 0:
            ax.legend(loc="upper right", ncol=4, fontsize=9)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"{title_prefix} — Foot velocity (world frame)")
    fig.tight_layout()
    fig.savefig(save_dir / "03_foot_velocity.png", dpi=130)
    plt.close(fig)


def plot_body_state(d: dict, title_prefix: str, save_dir: Path):
    if "gt_t" not in d:
        print("  skip body state plot: missing data")
        return
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    t = d["gt_t"]
    rpy = np.rad2deg(d["gt_rpy"])
    omg = d["gt_omega"]
    axes[0].plot(t, rpy[:, 0], label="roll",  color="C0", lw=1.0)
    axes[0].plot(t, rpy[:, 1], label="pitch", color="C2", lw=1.0)
    axes[0].plot(t, rpy[:, 2], label="yaw",   color="C3", lw=1.0)
    axes[0].set_ylabel("orientation [deg]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right", ncol=3)

    axes[1].plot(t, omg[:, 0], label="ωx", color="C0", lw=1.0)
    axes[1].plot(t, omg[:, 1], label="ωy", color="C2", lw=1.0)
    axes[1].plot(t, omg[:, 2], label="ωz", color="C3", lw=1.0)
    axes[1].set_ylabel("angular vel [rad/s]")
    axes[1].set_xlabel("t [s]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right", ncol=3)

    fig.suptitle(f"{title_prefix} — Body orientation and angular velocity")
    fig.tight_layout()
    fig.savefig(save_dir / "04_body_state.png", dpi=130)
    plt.close(fig)


def plot_contact_timing(d: dict, title_prefix: str, save_dir: Path):
    have_planned = "lp_t" in d
    have_actual = "fc_t" in d
    if not (have_planned or have_actual):
        print("  skip contact timing plot: no data")
        return

    fig, axes = plt.subplots(4, 1, figsize=(13, 9), sharex=True)
    for leg in range(4):
        ax = axes[leg]
        # Planned: solid step at y=1
        if have_planned:
            ax.step(d["lp_t"], d["lp_contact"][:, leg].astype(int),
                    where="post", color="C0", lw=1.6, label="planned")
        # Actual contact bit (offset to y=2 so they don't overlap)
        if have_actual:
            ax.step(d["fc_t"], d["fc_state"][:, leg].astype(int) + 2,
                    where="post", color="C3", lw=1.0, label="actual (sensor)")
            # Normalized raw foot force overlaid (scaled to [0,1])
            raw = d["fc_raw"][:, leg]
            raw_max = max(raw.max() if raw.size else 1.0, 1.0)
            ax.plot(d["fc_t"], raw / raw_max,
                    color="0.5", lw=0.6, alpha=0.6,
                    label="|F_foot| (norm.)")
        ax.set_yticks([0, 1, 2, 3])
        ax.set_yticklabels(["plan=0", "plan=1", "act=0", "act=1"])
        ax.set_ylim(-0.2, 3.2)
        ax.set_ylabel(LEG_NAMES[leg])
        ax.grid(True, alpha=0.3)
        if leg == 0:
            ax.legend(loc="upper right", ncol=3, fontsize=9)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(
        f"{title_prefix} — Contact timing: planned (local_plan) vs actual "
        "(foot_force sensor)")
    fig.tight_layout()
    fig.savefig(save_dir / "05_contact_timing.png", dpi=130)
    plt.close(fig)


def process_bag(bag_path: str, out_root: Path) -> Path:
    name = Path(bag_path).name
    print(f"\n=== Processing {name} ===")
    save_dir = out_root / name
    save_dir.mkdir(parents=True, exist_ok=True)
    data = parse_bag(bag_path)

    # Quick diagnostics
    for k, v in data.items():
        if isinstance(v, np.ndarray):
            print(f"  {k}: shape={v.shape}")

    plot_grfs(data, name, save_dir)
    plot_torques(data, name, save_dir)
    plot_foot_velocity(data, name, save_dir)
    plot_body_state(data, name, save_dir)
    plot_contact_timing(data, name, save_dir)
    print(f"  -> saved to {save_dir}")
    return save_dir


def main():
    bags_root = Path(
        "/home/rml/ros2_ws/src/quad-sdk/quad_logger/bags")
    out_root = bags_root / "plots"
    out_root.mkdir(parents=True, exist_ok=True)

    if len(sys.argv) > 1:
        bag_paths = [str(Path(p)) for p in sys.argv[1:]]
    else:
        bag_paths = sorted([
            str(p) for p in bags_root.iterdir()
            if p.is_dir()
            and p.name.startswith("robot_1_quad_log_go2_")
            and (p / "metadata.yaml").exists()
        ])

    for bp in bag_paths:
        process_bag(bp, out_root)


if __name__ == "__main__":
    main()
