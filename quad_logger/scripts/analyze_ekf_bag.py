"""One-off analysis: comp_filter (ground_truth) vs EKF (estimate) + gravity check.

Reads an mcap bag with rosbag2_py, compares the two estimators, and runs the
key diagnostic: at a standing-still window, compute u = R*fk + g from the IMU
and an orientation, to see whether gravity is being removed (u~0) or leaking
(u_horiz ~ g*sin(pitch)).
"""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1] if len(sys.argv) > 1 else \
    "/root/ros2_ws/src/quad-sdk/quad_logger/robot_1_quad_log_go2_20260521_2302"
GT = "/robot_1/state/ground_truth"
EST = "/robot_1/state/estimate"
IMU = "/robot_1/state/imu"


def quat_to_rpy(w, x, y, z):
    # roll, pitch, yaw (ZYX)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def R_world_body(w, x, y, z):
    n = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y - w*z),   2*(x*z + w*y)],
        [2*(x*y + w*z),   1 - 2*(x*x+z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),   2*(y*z + w*x),   1 - 2*(x*x+y*y)],
    ])


def read():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    tm = {t.name: t.type for t in r.get_all_topics_and_types()}
    msgs = {GT: [], EST: [], IMU: []}
    types = {k: get_message(tm[k]) for k in msgs if k in tm}
    while r.has_next():
        n, b, t = r.read_next()
        if n in msgs:
            msgs[n].append((t * 1e-9, deserialize_message(b, types[n])))
    return msgs


def state_arrays(rows):
    t, pos, vel, rpy = [], [], [], []
    for ts, m in rows:
        t.append(ts)
        pos.append([m.body.pose.position.x, m.body.pose.position.y,
                    m.body.pose.position.z])
        vel.append([m.body.twist.linear.x, m.body.twist.linear.y,
                    m.body.twist.linear.z])
        o = m.body.pose.orientation
        rpy.append(quat_to_rpy(o.w, o.x, o.y, o.z))
    return (np.array(t), np.array(pos), np.array(vel), np.array(rpy))


def main():
    print(f"Bag: {BAG}\n")
    msgs = read()
    for k in (GT, EST, IMU):
        print(f"  {k}: {len(msgs[k])} msgs")
    if not msgs[GT] or not msgs[EST]:
        print("Missing GT or EST topic."); return

    gt_t, gt_p, gt_v, gt_rpy = state_arrays(msgs[GT])
    es_t, es_p, es_v, es_rpy = state_arrays(msgs[EST])
    t0 = max(gt_t[0], es_t[0])
    gt_t -= t0; es_t -= t0

    print("\n=== INITIAL POSITION (first sample of each) ===")
    print(f"  comp filter (mocap):  ({gt_p[0,0]:+.3f}, {gt_p[0,1]:+.3f}, {gt_p[0,2]:+.3f})")
    print(f"  EKF estimate:         ({es_p[0,0]:+.3f}, {es_p[0,1]:+.3f}, {es_p[0,2]:+.3f})")
    print(f"  init offset:          ({es_p[0,0]-gt_p[0,0]:+.3f}, "
          f"{es_p[0,1]-gt_p[0,1]:+.3f}, {es_p[0,2]-gt_p[0,2]:+.3f})")

    # Velocity drift rate of EKF (linear fit over whole run)
    print("\n=== EKF velocity drift (linear fit slope = net accel bias) ===")
    for i, ax in enumerate("xyz"):
        a, b = np.polyfit(es_t, es_v[:, i], 1)
        print(f"  vel {ax}: slope={a:+.3f} m/s^2   v0={b:+.2f} m/s")

    # Gravity diagnostic on the cleanest standstill segment.
    print("\n=== GRAVITY CHECK at standstill (u = R*fk + g, should be ~0) ===")
    if msgs[IMU]:
        # Find the longest contiguous low-speed segment using gyro magnitude
        # from the IMU (independent of any estimator).
        imu_t = np.array([ts for ts, _ in msgs[IMU]]) - t0
        gyro = np.array([[m.angular_velocity.x, m.angular_velocity.y,
                          m.angular_velocity.z] for _, m in msgs[IMU]])
        acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y,
                         m.linear_acceleration.z] for _, m in msgs[IMU]])
        wmag = np.linalg.norm(gyro, axis=1)
        still = wmag < 0.05  # rad/s, essentially not rotating
        # longest run of True
        best_lo = best_len = cur_lo = cur_len = 0
        for i, s in enumerate(still):
            if s:
                if cur_len == 0:
                    cur_lo = i
                cur_len += 1
                if cur_len > best_len:
                    best_len, best_lo = cur_len, cur_lo
            else:
                cur_len = 0
        sl = slice(best_lo, best_lo + best_len)
        t_lo, t_hi = imu_t[best_lo], imu_t[best_lo + best_len - 1]
        fk = acc[sl].mean(axis=0)
        print(f"  still window: {t_lo:.2f}-{t_hi:.2f}s ({best_len} samples, gyro<0.05)")
        # orientation from comp filter (mocap) nearest window center
        ci = np.argmin(np.abs(gt_t - 0.5 * (t_lo + t_hi)))
        o = msgs[GT][ci][1].body.pose.orientation
        roll, pitch, yaw = quat_to_rpy(o.w, o.x, o.y, o.z)
        R = R_world_body(o.w, o.x, o.y, o.z)
        g = np.array([0, 0, -9.81])
        u = R @ fk + g
        print(f"  mocap roll/pitch/yaw = ({np.degrees(roll):.2f}, "
              f"{np.degrees(pitch):.2f}, {np.degrees(yaw):.2f}) deg")
        print(f"  mean IMU accel (body): ({fk[0]:+.3f}, {fk[1]:+.3f}, {fk[2]:+.3f})  |f|={np.linalg.norm(fk):.3f} m/s^2")
        print(f"    -> |f| should be 9.81; deficit = {9.81-np.linalg.norm(fk):+.3f} m/s^2 ({100*(np.linalg.norm(fk)-9.81)/9.81:+.1f}%)")
        print(f"  u = R*fk + g (world):  ({u[0]:+.3f}, {u[1]:+.3f}, {u[2]:+.3f}) m/s^2   <- should be ~0")
        print(f"    |u_horiz|={np.hypot(u[0],u[1]):.3f}   g*sin(pitch)={9.81*np.sin(abs(pitch)):.3f}   4%*g={0.04*9.81:.3f}")
    else:
        print("  no IMU messages in bag")


if __name__ == "__main__":
    main()
