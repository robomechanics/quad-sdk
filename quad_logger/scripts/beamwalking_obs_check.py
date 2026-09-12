#!/usr/bin/env python3
"""Compare the beamwalking policy's logged observations against Isaac rollouts.

Reads the beamwalking/policy_debug topic (68 obs + 12 raw + 12 clipped actions
+ tick + stamp) from a quad-sdk bag and prints per-channel statistics over the
steady walking window. Optionally compares against an Isaac evaluation .npz
(results/validation_*/trial_*.npz) for the channels that file records.

Usage:
  beamwalking_obs_check.py <bag_dir> [--isaac trial.npz] [--start S --end E]
"""
import argparse
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

OBS_NAMES = (
    [f"lin_vel_{a}" for a in "xyz"] + [f"ang_vel_{a}" for a in "xyz"]
    + [f"gravity_{a}" for a in "xyz"]
    + [f"q_rel_{j}" for j in ("FLh", "FRh", "RLh", "RRh", "FLt", "FRt", "RLt",
                               "RRt", "FLc", "FRc", "RLc", "RRc")]
    + [f"qd_{j}" for j in ("FLh", "FRh", "RLh", "RRh", "FLt", "FRt", "RLt",
                            "RRt", "FLc", "FRc", "RLc", "RRc")]
    + [f"prev_act_{i}" for i in range(12)]
    + [f"sin_phase_{l}" for l in ("FL", "FR", "RL", "RR")]
    + [f"cos_phase_{l}" for l in ("FL", "FR", "RL", "RR")]
    + ["cmd_speed", "cmd_duty", "cmd_width", "cmd_period", "gait_trot",
       "gait_walk"]
    + [f"desired_{l}" for l in ("FL", "FR", "RL", "RR")]
    + ["heading"]
    + [f"contact_{l}" for l in ("FL", "FR", "RL", "RR")]
)
ACT_NAMES = [f"act_{j}" for j in ("FLh", "FRh", "RLh", "RRh", "FLt", "FRt",
                                   "RLt", "RRt", "FLc", "FRc", "RLc", "RRc")]


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    dbg_topic = [t for t in types if t.endswith("beamwalking/policy_debug")]
    if not dbg_topic:
        sys.exit("bag has no beamwalking/policy_debug topic")
    reader.set_filter(rosbag2_py.StorageFilter(topics=dbg_topic))
    rows, stamps = [], []
    while reader.has_next():
        _, data, t = reader.read_next()
        msg = deserialize_message(data, get_message(types[dbg_topic[0]]))
        rows.append(msg.data)
        stamps.append(t * 1e-9)
    return np.array(stamps), np.array(rows)


def stats(name, x, ref=None):
    line = f"{name:14s} mean {x.mean():8.3f} std {x.std():7.3f} " \
           f"min {x.min():8.3f} max {x.max():8.3f}"
    if ref is not None:
        line += f" | isaac mean {ref.mean():8.3f} std {ref.std():7.3f}"
    print(line)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bag")
    ap.add_argument("--isaac", help="Isaac trial .npz for reference stats")
    ap.add_argument("--start", type=float, default=None,
                    help="window start, s after first inference")
    ap.add_argument("--end", type=float, default=None)
    args = ap.parse_args()

    t, d = read_bag(args.bag)
    t -= t[0]
    n_obs, n_act = 68, 12
    obs = d[:, :n_obs]
    raw = d[:, n_obs:n_obs + n_act]
    tick = d[:, n_obs + 2 * n_act]
    print(f"{len(t)} inferences over {t[-1]:.1f} s "
          f"({len(t) / max(t[-1], 1e-9):.1f} Hz)")
    # Default window: skip the first 3 s (walk start transient) and last 1 s.
    s = 3.0 if args.start is None else args.start
    e = t[-1] - 1.0 if args.end is None else args.end
    w = (t >= s) & (t <= e)
    print(f"window {s:.1f}-{e:.1f} s, {w.sum()} samples")
    obs, raw, tick = obs[w], raw[w], tick[w]

    ref = {}
    if args.isaac:
        z = np.load(args.isaac, allow_pickle=True)
        v = z["valid"]
        ref["lin_vel_x"] = z["body_forward_velocity"][v]
        ref["lin_vel_y"] = z["body_lateral_velocity"][v]
        ref["ang_vel_z"] = z["body_yaw_rate"][v]
        ref["heading"] = np.arctan2(
            2 * (z["root_quat"][..., 0] * z["root_quat"][..., 3]
                 + z["root_quat"][..., 1] * z["root_quat"][..., 2]),
            1 - 2 * (z["root_quat"][..., 2] ** 2 + z["root_quat"][..., 3] ** 2)
        )[v]
        for i, l in enumerate(("FL", "FR", "RL", "RR")):
            ref[f"contact_{l}"] = z["contacts"][..., i][v].astype(float)
            ref[f"desired_{l}"] = z["desired"][..., i][v].astype(float)
        q = z["root_quat"][v]
        # projected gravity = R^T (0,0,-1) from wxyz quaternion
        ref["gravity_x"] = -2 * (q[:, 1] * q[:, 3] - q[:, 0] * q[:, 2])
        ref["gravity_y"] = -2 * (q[:, 2] * q[:, 3] + q[:, 0] * q[:, 1])
        ref["gravity_z"] = -(q[:, 0] ** 2 - q[:, 1] ** 2 - q[:, 2] ** 2
                             + q[:, 3] ** 2)

    print("\n--- observations ---")
    for i, name in enumerate(OBS_NAMES):
        stats(name, obs[:, i], ref.get(name))

    print("\n--- raw actions (Isaac joint order) ---")
    for i, name in enumerate(ACT_NAMES):
        stats(name, raw[:, i])
    print(f"\nmax |raw action| {np.abs(raw).max():.2f} (clip 5)")

    # Phase clock sanity: ticks should advance by exactly one per inference
    # and wrap at period_ticks.
    dt = np.diff(tick)
    period = int(tick.max()) + 1
    bad = np.sum(~((dt == 1) | (dt == 1 - period)))
    print(f"phase clock: period {period} ticks, {bad} irregular steps")

    # Contact vs desired agreement, the training env's contact_accuracy metric.
    des = obs[:, 59:63]
    con = obs[:, 64:68]
    print(f"contact == desired: {np.mean(des == con):.3f}  "
          f"stance fraction {con.mean(0).round(2)}  "
          f"desired fraction {des.mean(0).round(2)}")


if __name__ == "__main__":
    main()
