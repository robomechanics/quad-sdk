#!/usr/bin/env python3
"""Commanded vs. actual joint angle plots for the underbrush swing controller.

The underbrush controller (robot_driver/src/controllers/underbrush_inverse_dynamics.cpp)
puts every leg into one of three modes on every tick:

  stance          pos_setpoint = planner joints, kp = stance_kp, torque_ff = ID torque
  swing (normal)  pos_setpoint = planner joints, kp = swing_kp,  torque_ff = 0
  swing (retract) kp = 0 on hip/knee, hip vel_setpoint = -retract_vel * hip_retract_sign,
                  knee torque_ff = -tau_push, abad still position controlled

None of that mode state is published, so this script re-derives it from the gains and
feedforward torques logged on control/joint_command, then lines the commands up against
the measured joint angles on state/ground_truth.

What each knob shows up as in the output:

  retract_vel  Hip velocity panel during the orange (retract) spans. The commanded
               step is retract_vel; if the measured hip velocity plateaus well below
               it, the motors are saturating and the extra command does nothing.
  t_down       "extend" column of the per-swing table: seconds between the end of the
               last retract span and touchdown, plus the position error still left at
               touchdown. Large touchdown error means the leg ran out of time to
               extend and t_down is too small.
  t_up         "1st retract" column: seconds from liftoff to the first retract span.
               Values pinned at t_up mean the contact trigger fired immediately at
               liftoff and t_up is the only thing holding it off.

Position tracking error is meaningless on the hip and knee inside a retract span
(kp = 0 there, and pos_setpoint is left at 0), so those samples are masked out of the
position error traces and the error statistics.

Usage:
    ./debug_underbrush_controller.py                        # newest bag in quad_logger/bags
    ./debug_underbrush_controller.py <bag.mcap|bag_dir>
    ./debug_underbrush_controller.py <bag> --legs 0 2 --t0 5 --t1 12
    ./debug_underbrush_controller.py <bag> --save out/underbrush --no-show
"""

import argparse
import glob
import os
import sys
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

LEG_NAMES = ["FL", "RL", "FR", "RR"]
JOINT_NAMES = ["abad", "hip", "knee"]

MODE_STANCE, MODE_SWING, MODE_RETRACT = 0, 1, 2
MODE_LABELS = {MODE_STANCE: "stance", MODE_SWING: "swing", MODE_RETRACT: "retract"}
MODE_COLORS = {MODE_STANCE: "0.85", MODE_SWING: "#cfe3f7", MODE_RETRACT: "#fbdcc0"}


def find_bag(path):
    """Resolve a user-supplied path (or None) to a sorted list of .mcap files."""
    if path is None:
        bag_root = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "bags")
        candidates = sorted(glob.glob(os.path.join(bag_root, "**", "*.mcap"),
                                      recursive=True), key=os.path.getmtime)
        if not candidates:
            sys.exit(f"No .mcap files found under {os.path.normpath(bag_root)}")
        newest = candidates[-1]
        print(f"No bag given, using newest: {newest}")
        return [newest]

    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, "*.mcap")))
        if not files:
            sys.exit(f"No .mcap files in {path}")
        return files

    if not os.path.exists(path):
        sys.exit(f"{path} does not exist")
    return [path]


def stamp_seconds(header):
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def read_bag(files, namespace):
    """Pull the joint command, measured state and foot contact streams out of a bag."""
    topics = {
        "cmd": f"/{namespace}/control/joint_command",
        "state": f"/{namespace}/state/ground_truth",
        "contact": f"/{namespace}/state/foot_contact",
    }
    raw = defaultdict(list)

    available = set()
    for path in files:
        with open(path, "rb") as f:
            summary = make_reader(f).get_summary()
            available.update(ch.topic for ch in summary.channels.values())

    for key, topic in topics.items():
        if topic not in available:
            if key == "contact":
                print(f"Note: {topic} not in bag, touchdown markers disabled")
            else:
                sys.exit(f"Required topic {topic} not in bag. Topics present:\n  "
                         + "\n  ".join(sorted(available)))

    wanted = [t for t in topics.values() if t in available]
    for path in files:
        with open(path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _, channel, _, msg in reader.iter_decoded_messages(topics=wanted):
                raw[channel.topic].append(msg)

    cmd = parse_commands(raw[topics["cmd"]])
    state = parse_state(raw[topics["state"]])
    contact = parse_contact(raw.get(topics["contact"], []))
    return cmd, state, contact


def parse_commands(msgs):
    """LegCommandArray stream -> arrays shaped (N, 4, 3)."""
    if not msgs:
        sys.exit("No messages on the joint command topic")
    out = {"time": np.array([stamp_seconds(m.header) for m in msgs])}
    fields = ["pos_setpoint", "vel_setpoint", "kp", "kd", "torque_ff"]
    for field in fields:
        out[field] = np.array(
            [[[getattr(m.leg_commands[i].motor_commands[j], field)
               for j in range(3)] for i in range(4)] for m in msgs])
    return out


def parse_state(msgs):
    """RobotState stream -> joint arrays shaped (N, 4, 3) in leg-major order.

    Foot height is carried alongside because joint-space error is not a usable tuning
    signal during the extend phase: the controller restores the reference to the
    nominal plan the moment force_mode clears, so the retracted leg is compared
    against a trajectory it was never on. Clearance is measured instead.
    """
    if not msgs:
        sys.exit("No messages on the state topic")
    time = np.array([stamp_seconds(m.header) for m in msgs])
    pos = np.array([np.asarray(m.joints.position) for m in msgs]).reshape(-1, 4, 3)
    vel = np.array([np.asarray(m.joints.velocity) for m in msgs]).reshape(-1, 4, 3)
    eff = np.array([np.asarray(m.joints.effort) for m in msgs]).reshape(-1, 4, 3)
    foot_z = np.array([[f.position.z for f in m.feet.feet] for m in msgs])
    return {"time": time, "position": pos, "velocity": vel, "effort": eff,
            "foot_z": foot_z}


def parse_contact(msgs):
    if not msgs:
        return None
    return {
        "time": np.array([stamp_seconds(m.header) for m in msgs]),
        "states": np.array([list(m.contact_states) for m in msgs], dtype=bool),
    }


def classify_modes(cmd):
    """Recover stance / swing / retract per leg per tick from the logged gains.

    Retract is unambiguous: it is the only mode that zeroes the knee kp. Stance and
    normal swing are separated by knee kp when stance_kp != swing_kp, and fall back to
    the feedforward torque (zero in swing, inverse-dynamics output in stance) when the
    config uses the same gain for both.
    """
    knee_kp = cmd["kp"][:, :, 2]
    knee_tff = cmd["torque_ff"][:, :, 2]

    modes = np.full(knee_kp.shape, MODE_SWING, dtype=int)
    retract = knee_kp <= 1e-9
    gains = np.unique(np.round(knee_kp[~retract], 6)) if (~retract).any() else np.array([])

    if gains.size >= 2:
        thresh = 0.5 * (gains.min() + gains.max())
        stance = (~retract) & (knee_kp > thresh)
        print(f"Mode split from knee kp: stance={gains.max():g}, swing={gains.min():g}")
    else:
        stance = (~retract) & (np.abs(knee_tff) > 1e-9)
        print("Stance and swing gains are identical; splitting on knee torque_ff")

    modes[stance] = MODE_STANCE
    modes[retract] = MODE_RETRACT
    return modes


def resample(state, t_query):
    """Linearly interpolate the measured joint signals onto the command timestamps."""
    out = {}
    for key in ("position", "velocity", "effort"):
        arr = np.empty((t_query.size, 4, 3))
        for i in range(4):
            for j in range(3):
                arr[:, i, j] = np.interp(t_query, state["time"], state[key][:, i, j])
        out[key] = arr
    foot_z = np.empty((t_query.size, 4))
    for i in range(4):
        foot_z[:, i] = np.interp(t_query, state["time"], state["foot_z"][:, i])
    out["foot_z"] = foot_z
    return out


def contiguous_runs(mask):
    """Yield (start, stop) index pairs for each True run in a 1-D boolean mask."""
    if not mask.any():
        return []
    edges = np.diff(mask.astype(np.int8))
    starts = list(np.flatnonzero(edges == 1) + 1)
    stops = list(np.flatnonzero(edges == -1) + 1)
    if mask[0]:
        starts.insert(0, 0)
    if mask[-1]:
        stops.append(mask.size)
    return list(zip(starts, stops))


def swing_cycles(time, modes_leg):
    """Split one leg's mode trace into swings, with retract spans inside each."""
    cycles = []
    for start, stop in contiguous_runs(modes_leg != MODE_STANCE):
        if start == 0 or stop == modes_leg.size:
            continue  # clipped by the ends of the log; timings would be wrong
        retracts = [(start + a, start + b) for a, b in
                    contiguous_runs(modes_leg[start:stop] == MODE_RETRACT)]
        cycles.append({
            "start": start,
            "stop": stop,
            "t_lo": time[start],
            "t_td": time[stop],
            "retracts": retracts,
        })
    return cycles


def shade_modes(ax, time, modes_leg):
    for mode in (MODE_STANCE, MODE_SWING, MODE_RETRACT):
        for start, stop in contiguous_runs(modes_leg == mode):
            ax.axvspan(time[start], time[min(stop, time.size - 1)],
                       color=MODE_COLORS[mode], lw=0, zorder=0)


def touchdown_times(contact, leg, t0, t1):
    """Rising edges of the measured foot contact flag, for overlay on the plots."""
    if contact is None:
        return np.array([])
    flag = contact["states"][:, leg].astype(np.int8)
    rising = np.flatnonzero(np.diff(flag) == 1) + 1
    t = contact["time"][rising]
    return t[(t >= t0) & (t <= t1)]


def plot_leg(leg, time, cmd, meas, modes, contact, t_start, t0, t1):
    """One figure per leg: position and velocity tracking for all three joints."""
    fig, axes = plt.subplots(3, 2, figsize=(15, 9), sharex=True)
    fig.suptitle(f"Leg {leg} ({LEG_NAMES[leg]}) - commanded vs actual "
                 f"(shading: grey stance, blue swing, orange retract)")

    td = touchdown_times(contact, leg, t_start + t0, t_start + t1) - t_start
    pos_valid = position_valid_mask(modes)

    for j in range(3):
        ax_p, ax_v = axes[j, 0], axes[j, 1]
        shade_modes(ax_p, time, modes[:, leg])
        shade_modes(ax_v, time, modes[:, leg])

        cmd_pos = np.where(pos_valid[:, leg, j], cmd["pos_setpoint"][:, leg, j], np.nan)
        ax_p.plot(time, cmd_pos, "--", color="tab:red", lw=1.4, label="commanded")
        ax_p.plot(time, meas["position"][:, leg, j], "-", color="tab:blue", lw=1.2,
                  label="actual")
        ax_p.set_ylabel(f"{JOINT_NAMES[j]} [rad]")

        ax_v.plot(time, cmd["vel_setpoint"][:, leg, j], "--", color="tab:red", lw=1.4,
                  label="commanded")
        ax_v.plot(time, meas["velocity"][:, leg, j], "-", color="tab:blue", lw=1.2,
                  label="actual")
        ax_v.set_ylabel(f"{JOINT_NAMES[j]} [rad/s]")

        for ax in (ax_p, ax_v):
            for t in td:
                ax.axvline(t, color="k", ls=":", lw=0.8, zorder=1)
            ax.grid(alpha=0.3)
            ax.set_xlim(t0, t1)

    axes[0, 0].set_title("joint position (dotted black = measured touchdown)")
    axes[0, 1].set_title("joint velocity")
    axes[0, 0].legend(loc="upper right", fontsize=8)
    axes[2, 0].set_xlabel("time [s]")
    axes[2, 1].set_xlabel("time [s]")
    fig.tight_layout()
    return fig


def position_valid_mask(modes):
    """True where pos_setpoint is actually being tracked.

    In retract the controller zeroes hip and knee kp and leaves pos_setpoint at 0, so
    comparing those two samples against the measurement is meaningless. abad stays
    position controlled in every mode.
    """
    valid = np.ones((modes.shape[0], 4, 3), dtype=bool)
    retract = modes == MODE_RETRACT
    valid[:, :, 1] = ~retract
    valid[:, :, 2] = ~retract
    return valid


def plot_errors(time, cmd, meas, modes, legs, t0, t1):
    """One 4x3 grid of position tracking error, error = actual - commanded."""
    fig, axes = plt.subplots(4, 3, figsize=(15, 10), sharex=True)
    fig.suptitle("Joint position error (actual - commanded); "
                 "positive = actual is ahead of target")
    pos_valid = position_valid_mask(modes)
    err = meas["position"] - cmd["pos_setpoint"]
    err = np.where(pos_valid, err, np.nan)

    for i in range(4):
        for j in range(3):
            ax = axes[i, j]
            if i in legs:
                shade_modes(ax, time, modes[:, i])
                ax.plot(time, err[:, i, j], color="tab:purple", lw=1.0)
            ax.axhline(0.0, color="k", lw=0.8)
            ax.grid(alpha=0.3)
            ax.set_xlim(t0, t1)
            if j == 0:
                ax.set_ylabel(f"{LEG_NAMES[i]} [rad]")
            if i == 0:
                ax.set_title(JOINT_NAMES[j])
            if i == 3:
                ax.set_xlabel("time [s]")
    fig.tight_layout()
    return fig


def plot_swing_summary(cycles_by_leg, t_start, legs):
    """Per-swing scatter of the three numbers the tuning knobs move."""
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle("Per-swing diagnostics (x = liftoff time)")

    for leg in legs:
        cycles = cycles_by_leg[leg]
        if not cycles:
            continue
        t = [c["t_lo"] - t_start for c in cycles]
        axes[0].plot(t, [c["hip_vel_ratio"] for c in cycles], "o-", ms=4,
                     label=LEG_NAMES[leg])
        axes[1].plot(t, [c["extend_time"] for c in cycles], "o-", ms=4,
                     label=LEG_NAMES[leg])
        axes[2].plot(t, [c["apex"] for c in cycles], "o-", ms=4,
                     label=LEG_NAMES[leg])

    axes[0].axhline(1.0, color="k", ls="--", lw=0.8)
    axes[0].set_ylabel("hip |vel| achieved\n/ retract_vel")
    axes[0].set_title("retract_vel: below 1.0 means the hip never reaches the command")
    axes[1].set_ylabel("extend time [s]")
    axes[1].set_title("t_down: seconds from end of retract to touchdown")
    axes[2].set_ylabel("apex clearance [m]")
    axes[2].set_title("retract_vel: foot lift above liftoff height. "
                      "A rising trend means the extend phase cannot undo the retract")
    axes[2].set_xlabel("time [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, ncol=4)
    fig.tight_layout()
    return fig


def analyze_swings(time, cmd, meas, modes, legs):
    """Per-swing timing and tracking numbers, keyed by leg."""
    cycles_by_leg = {}
    err = meas["position"] - cmd["pos_setpoint"]

    for leg in legs:
        cycles = swing_cycles(time, modes[:, leg])
        for c in cycles:
            start, stop = c["start"], c["stop"]
            c["duration"] = c["t_td"] - c["t_lo"]

            if c["retracts"]:
                r0, r1 = c["retracts"][0][0], c["retracts"][-1][1]
                c["t_first_retract"] = time[r0] - c["t_lo"]
                c["retract_time"] = sum(time[min(b, time.size - 1)] - time[a]
                                        for a, b in c["retracts"])
                c["extend_time"] = c["t_td"] - time[min(r1, time.size - 1)]
                idx = np.concatenate([np.arange(a, b) for a, b in c["retracts"]])
                cmd_vel = np.abs(cmd["vel_setpoint"][idx, leg, 1])
                act_vel = np.abs(meas["velocity"][idx, leg, 1])
                c["hip_vel_cmd"] = float(np.median(cmd_vel))
                c["hip_vel_act"] = float(np.percentile(act_vel, 90))
                c["hip_vel_ratio"] = (c["hip_vel_act"] / c["hip_vel_cmd"]
                                      if c["hip_vel_cmd"] > 1e-9 else np.nan)
            else:
                c["t_first_retract"] = np.nan
                c["retract_time"] = 0.0
                c["extend_time"] = c["duration"]
                c["hip_vel_cmd"] = np.nan
                c["hip_vel_act"] = np.nan
                c["hip_vel_ratio"] = np.nan

            # Error on the last swing sample, i.e. what the leg lands with.
            td_err = err[stop - 1, leg, :]
            c["td_err"] = td_err
            c["td_err_norm"] = float(np.linalg.norm(td_err))

            # Foot clearance is the tuning signal for retract_vel and t_down: how far
            # the retraction lifted the foot, and whether the extend phase put it back
            # down. A clearance that grows swing over swing means the retraction is
            # larger than the extend window can undo.
            z = meas["foot_z"][start:stop, leg]
            c["apex"] = float(z.max() - z[0])
            c["land_z"] = float(z[-1] - z[0])
        cycles_by_leg[leg] = cycles
    return cycles_by_leg


def print_report(time, cmd, meas, modes, cycles_by_leg, legs):
    err = meas["position"] - cmd["pos_setpoint"]
    pos_valid = position_valid_mask(modes)

    print("\n" + "=" * 78)
    print("Position tracking error by mode (actual - commanded), rad")
    print("=" * 78)
    print(f"{'leg':<5}{'mode':<10}{'joint':<7}{'mean':>10}{'rms':>10}"
          f"{'p95 |err|':>12}{'max |err|':>12}")
    for leg in legs:
        for mode in (MODE_STANCE, MODE_SWING, MODE_RETRACT):
            for j in range(3):
                sel = (modes[:, leg] == mode) & pos_valid[:, leg, j]
                if sel.sum() < 2:
                    continue
                e = err[sel, leg, j]
                print(f"{LEG_NAMES[leg]:<5}{MODE_LABELS[mode]:<10}"
                      f"{JOINT_NAMES[j]:<7}{e.mean():>10.4f}"
                      f"{np.sqrt(np.mean(e ** 2)):>10.4f}"
                      f"{np.percentile(np.abs(e), 95):>12.4f}"
                      f"{np.abs(e).max():>12.4f}")

    print("\n" + "=" * 100)
    print("Per-swing timing. 1st retract vs t_up, extend vs t_down, "
          "hip vel vs retract_vel")
    print("=" * 100)
    print(f"{'leg':<5}{'t_lo':>9}{'swing':>8}{'1st retr':>10}{'retract':>9}"
          f"{'extend':>9}{'hip cmd':>9}{'hip act':>9}{'ratio':>7}"
          f"{'apex':>8}{'land z':>8}"
          f"{'abad':>8}{'hip':>8}{'knee':>8}")
    for leg in legs:
        for c in cycles_by_leg[leg]:
            print(f"{LEG_NAMES[leg]:<5}{c['t_lo'] - time[0]:>9.3f}"
                  f"{c['duration']:>8.3f}{c['t_first_retract']:>10.3f}"
                  f"{c['retract_time']:>9.3f}{c['extend_time']:>9.3f}"
                  f"{c['hip_vel_cmd']:>9.2f}{c['hip_vel_act']:>9.2f}"
                  f"{c['hip_vel_ratio']:>7.2f}"
                  f"{c['apex']:>8.3f}{c['land_z']:>8.3f}"
                  f"{c['td_err'][0]:>8.3f}{c['td_err'][1]:>8.3f}"
                  f"{c['td_err'][2]:>8.3f}")

    print("\n" + "=" * 78)
    print("Summary across swings (medians)")
    print("=" * 78)
    for leg in legs:
        cycles = cycles_by_leg[leg]
        if not cycles:
            print(f"{LEG_NAMES[leg]}: no complete swings in window")
            continue
        with np.errstate(invalid="ignore"):
            ratio = np.nanmedian([c["hip_vel_ratio"] for c in cycles])
            first = np.nanmedian([c["t_first_retract"] for c in cycles])
        extend = np.median([c["extend_time"] for c in cycles])
        apex = np.array([c["apex"] for c in cycles])
        n_retract = sum(1 for c in cycles if c["retracts"])
        # A positive slope means each retract lifts more than the extend puts back,
        # so the leg ratchets upward and the gait eventually breaks down.
        trend = (np.polyfit(np.arange(apex.size), apex, 1)[0]
                 if apex.size >= 3 else np.nan)
        flag = " RATCHETING" if trend > 0.01 else ""
        print(f"{LEG_NAMES[leg]}: {len(cycles)} swings ({n_retract} with retract) | "
              f"hip vel ratio {ratio:.2f} | first retract {first:.3f} s | "
              f"extend {extend:.3f} s | apex {np.median(apex):.3f} m "
              f"({trend:+.3f} m/swing){flag}")
    print()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", nargs="?", default=None,
                        help="path to a .mcap file or a bag directory "
                             "(default: newest bag under quad_logger/bags)")
    parser.add_argument("--namespace", default="robot_1", help="robot namespace")
    parser.add_argument("--legs", type=int, nargs="+", default=[0, 1, 2, 3],
                        choices=[0, 1, 2, 3],
                        help="legs to plot: 0=FL 1=RL 2=FR 3=RR")
    parser.add_argument("--t0", type=float, default=None,
                        help="window start, seconds from the beginning of the log")
    parser.add_argument("--t1", type=float, default=None,
                        help="window end, seconds from the beginning of the log")
    parser.add_argument("--save", default=None,
                        help="prefix to save figures to, e.g. out/underbrush")
    parser.add_argument("--no-show", action="store_true",
                        help="do not open the figure windows")
    args = parser.parse_args()

    files = find_bag(args.bag)
    cmd, state, contact = read_bag(files, args.namespace)

    t_start = cmd["time"][0]
    lo = t_start + args.t0 if args.t0 is not None else cmd["time"][0]
    hi = t_start + args.t1 if args.t1 is not None else cmd["time"][-1]
    keep = (cmd["time"] >= lo) & (cmd["time"] <= hi)
    if keep.sum() < 2:
        sys.exit("Time window contains fewer than 2 command samples")
    for key in ("time", "pos_setpoint", "vel_setpoint", "kp", "kd", "torque_ff"):
        cmd[key] = cmd[key][keep]

    print(f"{cmd['time'].size} command samples over "
          f"{cmd['time'][-1] - cmd['time'][0]:.2f} s, "
          f"{state['time'].size} state samples")

    modes = classify_modes(cmd)
    meas = resample(state, cmd["time"])
    cycles_by_leg = analyze_swings(cmd["time"], cmd, meas, modes, args.legs)
    print_report(cmd["time"], cmd, meas, modes, cycles_by_leg, args.legs)

    # Plot against seconds since the first command sample; epoch seconds make the
    # axis unreadable.
    t_rel = cmd["time"] - t_start
    t0, t1 = t_rel[0], t_rel[-1]
    figs = {}
    for leg in args.legs:
        figs[f"leg{leg}_{LEG_NAMES[leg]}"] = plot_leg(
            leg, t_rel, cmd, meas, modes, contact, t_start, t0, t1)
    figs["error"] = plot_errors(t_rel, cmd, meas, modes, args.legs, t0, t1)
    figs["swing_summary"] = plot_swing_summary(cycles_by_leg, t_start, args.legs)

    if args.save:
        out_dir = os.path.dirname(os.path.abspath(args.save))
        os.makedirs(out_dir, exist_ok=True)
        for name, fig in figs.items():
            path = f"{args.save}_{name}.png"
            fig.savefig(path, dpi=130)
            print(f"Saved {path}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
