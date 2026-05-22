"""Estimate IMU accel + gyro bias from a static (still) bag segment.

Record the robot powered and completely still (ideally level), e.g.:
  ros2 bag record -o imu_cal /robot_1/state/imu /robot_1/state/ground_truth

Then:
  python3 calibrate_imu_bias.py <bag_dir>

Prints robot_driver.yaml-ready bias_x/y/z (accel) and bias_r/p/w (gyro).
These are subtracted in EKFEstimator::StepOnce (fk -= bias, wk -= bias), so the
reported values are exactly what to paste.

Gyro bias  = mean(gyro)                       (true rate is 0 at rest)
Accel bias = mean(accel) - R_bw * (0,0,9.81)  (remove gravity in body frame)
If ground_truth orientation is present it is used for R_bw (handles non-level);
otherwise the robot is assumed level (R = I).
"""
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

IMU = "/robot_1/state/imu"
GT = "/robot_1/state/ground_truth"
G = 9.81


def R_world_body(w, x, y, z):
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("", ""))
    tm = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {IMU: [], GT: []}
    types = {k: get_message(tm[k]) for k in out if k in tm}
    while r.has_next():
        n, b, t = r.read_next()
        if n in types:
            out[n].append(deserialize_message(b, types[n]))
    return out


def main():
    if len(sys.argv) < 2:
        print("usage: calibrate_imu_bias.py <bag_dir>")
        return
    bag = sys.argv[1]
    msgs = read(bag)
    imu = msgs[IMU]
    if not imu:
        print(f"No {IMU} messages in bag")
        return

    acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y,
                     m.linear_acceleration.z] for m in imu])
    gyro = np.array([[m.angular_velocity.x, m.angular_velocity.y,
                      m.angular_velocity.z] for m in imu])

    # Use only the still part: reject samples where gyro magnitude is high.
    wmag = np.linalg.norm(gyro, axis=1)
    still = wmag < 0.05
    if still.sum() < 10:
        print(f"WARNING: only {still.sum()} still samples (gyro<0.05); "
              "using all samples. Make sure the robot was truly still.")
        still = np.ones(len(imu), dtype=bool)
    acc_s = acc[still]
    gyro_s = gyro[still]
    print(f"Static samples used: {still.sum()} / {len(imu)}")

    # Gyro bias = mean rate at rest.
    gyro_bias = gyro_s.mean(axis=0)

    # Gravity direction in body frame: use mocap orientation if available.
    gt = msgs[GT]
    if gt:
        o = gt[len(gt) // 2].body.pose.orientation
        R_wb = R_world_body(o.w, o.x, o.y, o.z)
        g_body = R_wb.T @ np.array([0.0, 0.0, G])
        roll = np.degrees(np.arctan2(R_wb[2, 1], R_wb[2, 2]))
        pitch = np.degrees(-np.arcsin(np.clip(R_wb[2, 0], -1, 1)))
        print(f"Orientation from ground_truth: roll={roll:.2f} pitch={pitch:.2f} deg")
    else:
        g_body = np.array([0.0, 0.0, G])
        print("No ground_truth orientation -> assuming LEVEL (roll=pitch=0).")

    accel_mean = acc_s.mean(axis=0)
    accel_bias = accel_mean - g_body

    print(f"\n  mean accel (body) = [{accel_mean[0]:+.4f} {accel_mean[1]:+.4f} "
          f"{accel_mean[2]:+.4f}]  |a|={np.linalg.norm(accel_mean):.4f}")
    print(f"  expected (gravity) = [{g_body[0]:+.4f} {g_body[1]:+.4f} "
          f"{g_body[2]:+.4f}]")
    print(f"  accel std (noise)  = [{acc_s.std(axis=0)[0]:.4f} "
          f"{acc_s.std(axis=0)[1]:.4f} {acc_s.std(axis=0)[2]:.4f}]")

    print("\n--- paste into robot_driver.yaml (robot_driver:) ---")
    print(f"      bias_x: {accel_bias[0]:.4f}")
    print(f"      bias_y: {accel_bias[1]:.4f}")
    print(f"      bias_z: {accel_bias[2]:.4f}")
    print(f"      bias_r: {gyro_bias[0]:.5f}")
    print(f"      bias_p: {gyro_bias[1]:.5f}")
    print(f"      bias_w: {gyro_bias[2]:.5f}")
    print("----------------------------------------------------")


if __name__ == "__main__":
    main()
