#!/usr/bin/env python3
"""Extract robot state data from a ROS2 bag (MCAP) into a JSON file for Blender.

Usage (must be run in a sourced ROS2 workspace so quad_msgs is importable):
    source install/setup.bash
    python3 scripts/extract_bag.py <bag_dir> [-o output.json] [-t topic]

The output JSON has the structure:
{
  "fps": <float>,
  "frames": [
    {
      "t": <float>,                          # seconds from start
      "pos": [x, y, z],                      # body position
      "quat": [x, y, z, w],                  # body orientation
      "joints": {"FL_hip_joint": val, ...}    # joint angles (rad)
    },
    ...
  ]
}
"""

import argparse
import json
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


# Quad-SDK joint name order (matches JointState.position indices)
JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]


def read_bag(bag_path, topic):
    """Yield deserialized messages for *topic* from the bag at *bag_path*."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    # Resolve the message type for the requested topic
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        available = "\n  ".join(topic_types.keys())
        print(f"Topic '{topic}' not in bag.  Available:\n  {available}",
              file=sys.stderr)
        sys.exit(1)

    msg_type_str = topic_types[topic]
    msg_type = get_message(msg_type_str)

    # Filter to only read the desired topic
    filter_ = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(filter_)

    while reader.has_next():
        _topic, data, _ts = reader.read_next()
        yield deserialize_message(data, msg_type)


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def extract(bag_path, topic):
    frames = []
    t0 = None

    for msg in read_bag(bag_path, topic):
        t = stamp_to_sec(msg.header.stamp)
        if t0 is None:
            t0 = t

        body = msg.body
        pos = [body.pose.position.x, body.pose.position.y, body.pose.position.z]
        quat = [
            body.pose.orientation.x,
            body.pose.orientation.y,
            body.pose.orientation.z,
            body.pose.orientation.w,
        ]

        joints = {}
        for i, name in enumerate(JOINT_NAMES):
            if i < len(msg.joints.position):
                joints[name] = msg.joints.position[i]
            else:
                joints[name] = 0.0

        frames.append({
            "t": t - t0,
            "pos": pos,
            "quat": quat,
            "joints": joints,
        })

    if len(frames) < 2:
        print("Not enough frames extracted", file=sys.stderr)
        sys.exit(1)

    # Compute average FPS from timestamps
    dt = frames[-1]["t"] / (len(frames) - 1)
    fps = 1.0 / dt if dt > 0 else 60.0

    return {"fps": round(fps, 2), "frames": frames}


def main():
    parser = argparse.ArgumentParser(
        description="Extract robot state from a ROS2 bag to JSON for Blender")
    parser.add_argument("bag", help="Path to the rosbag2 directory")
    parser.add_argument("-o", "--output", default=None,
                        help="Output JSON path (default: <bag_name>.json)")
    parser.add_argument("-t", "--topic",
                        default="/robot_1/state/ground_truth",
                        help="RobotState topic to extract")
    args = parser.parse_args()

    if args.output is None:
        import os
        bag_name = os.path.basename(args.bag.rstrip("/"))
        args.output = bag_name + ".json"

    print(f"Reading topic '{args.topic}' from '{args.bag}' ...")
    data = extract(args.bag, args.topic)
    print(f"Extracted {len(data['frames'])} frames at {data['fps']} FPS")

    with open(args.output, "w") as f:
        json.dump(data, f)
    print(f"Written to {args.output}")


if __name__ == "__main__":
    main()
