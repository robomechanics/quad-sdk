"""Plot EKF estimate vs ground truth for position and velocity."""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def load_bag(file_path):
    """Load ground truth and state estimate from an mcap file."""
    gt_msgs = []
    est_msgs = []
    gt_times = []
    est_times = []

    with open(file_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _, channel, message, ros_msg in reader.iter_decoded_messages(
            topics=["/robot_1/state/ground_truth",
                    "/robot_1/state/estimate"]
        ):
            t = message.log_time / 1e9
            if channel.topic == "/robot_1/state/ground_truth":
                gt_msgs.append(ros_msg)
                gt_times.append(t)
            elif channel.topic == "/robot_1/state/estimate":
                est_msgs.append(ros_msg)
                est_times.append(t)

    print(f"  Ground truth: {len(gt_msgs)} msgs")
    print(f"  State estimate: {len(est_msgs)} msgs")

    if len(gt_msgs) == 0:
        print("ERROR: No ground truth data found.")
        sys.exit(1)
    if len(est_msgs) == 0:
        print("ERROR: No state estimate data found on /robot_1/state/estimate.")
        print("Make sure the EKF was running and estimator_id was set to 'ekf_filter'.")
        sys.exit(1)

    def extract(msgs, times):
        pos = np.array([[m.body.pose.position.x,
                          m.body.pose.position.y,
                          m.body.pose.position.z] for m in msgs])
        vel = np.array([[m.body.twist.linear.x,
                          m.body.twist.linear.y,
                          m.body.twist.linear.z] for m in msgs])
        return np.array(times), pos, vel

    gt_t, gt_pos, gt_vel = extract(gt_msgs, gt_times)
    est_t, est_pos, est_vel = extract(est_msgs, est_times)

    # Time-align: use the later of the two start times as t=0
    t0 = max(gt_t[0], est_t[0])
    gt_t -= t0
    est_t -= t0

    # Trim to overlapping region
    t_end = min(gt_t[-1], est_t[-1])
    gt_mask = gt_t <= t_end
    est_mask = est_t <= t_end
    gt_t, gt_pos, gt_vel = gt_t[gt_mask], gt_pos[gt_mask], gt_vel[gt_mask]
    est_t, est_pos, est_vel = est_t[est_mask], est_pos[est_mask], est_vel[est_mask]

    print(f"  Overlapping window: {t_end:.1f}s "
          f"(gt: {np.sum(gt_mask)} msgs, est: {np.sum(est_mask)} msgs)")

    return gt_t, gt_pos, gt_vel, est_t, est_pos, est_vel


def plot_comparison(gt_t, gt_pos, gt_vel, est_t, est_pos, est_vel):
    labels = ['X', 'Y', 'Z']

    # Plot 1: Position
    fig1, axes1 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig1.canvas.manager.set_window_title("Position: Ground Truth vs EKF")
    for i, ax in enumerate(axes1):
        ax.plot(gt_t, gt_pos[:, i], 'b-', linewidth=1.5, label='Ground Truth')
        ax.plot(est_t, est_pos[:, i], 'r--', linewidth=1.5, label='EKF Estimate')
        ax.set_ylabel(f'{labels[i]} (m)')
        ax.grid(True)
        ax.legend(loc='upper right')
    axes1[0].set_title('Position: Ground Truth vs EKF Estimate')
    axes1[-1].set_xlabel('Time (s)')
    fig1.tight_layout()

    # Plot 2: Velocity
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig2.canvas.manager.set_window_title("Velocity: Ground Truth vs EKF")
    for i, ax in enumerate(axes2):
        ax.plot(gt_t, gt_vel[:, i], 'b-', linewidth=1.5, label='Ground Truth')
        ax.plot(est_t, est_vel[:, i], 'r--', linewidth=1.5, label='EKF Estimate')
        ax.set_ylabel(f'{labels[i]} (m/s)')
        ax.grid(True)
        ax.legend(loc='upper right')
    axes2[0].set_title('Velocity: Ground Truth vs EKF Estimate')
    axes2[-1].set_xlabel('Time (s)')
    fig2.tight_layout()

    plt.show()


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    bags_dir = os.path.join(script_dir, "..", "bags")

    if len(sys.argv) > 1:
        trial = sys.argv[1]
    else:
        trial = "robot_1_quad_log_go2_20260402_2308"

    mcap_path = os.path.join(bags_dir, trial, f"{trial}_0.mcap")
    if not os.path.exists(mcap_path):
        print(f"File not found: {mcap_path}")
        print("Usage: python plot_ekf_comparison.py [trial_name]")
        sys.exit(1)

    print(f"Loading {mcap_path}...")
    gt_t, gt_pos, gt_vel, est_t, est_pos, est_vel = load_bag(mcap_path)
    plot_comparison(gt_t, gt_pos, gt_vel, est_t, est_pos, est_vel)
