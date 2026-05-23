#!/usr/bin/env python3
"""
Waits for the robot to be fully ready in simulation:
  1. state/ground_truth is being published (Gazebo + driver running)
  2. state/joints is being published (controllers activated)
  3. Robot is upright (not spawned flipped)
  4. Robot has landed (vertical velocity near zero)

Only exits once ALL conditions hold for required_msgs consecutive
messages. Used by run_iteration.py to gate the stand command.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from quad_msgs.msg import RobotState


def quaternion_to_rp(q):
    """Convert geometry_msgs Quaternion to (roll, pitch) in radians."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    return roll, pitch


class WaitForRobotNode(Node):
    def __init__(self):
        super().__init__('wait_for_robot')

        self.declare_parameter('namespace', 'robot_1')
        self.declare_parameter('required_msgs', 50)
        self.declare_parameter('max_tilt', 0.5)       # radians (~29 deg)
        self.declare_parameter('max_vz', 0.5)          # m/s — must be below this to count as landed

        ns = self.get_parameter('namespace').value
        self.required_msgs = self.get_parameter('required_msgs').value
        self.max_tilt = self.get_parameter('max_tilt').value
        self.max_vz = self.get_parameter('max_vz').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub_state = self.create_subscription(
            RobotState, f'/{ns}/state/ground_truth', self.state_cb, qos)
        self.sub_joints = self.create_subscription(
            JointState, f'/{ns}/joint_states', self.joints_cb, qos)

        self.ready_pub = self.create_publisher(Bool, f'/{ns}/robot_ready', 10)

        self.joints_received = False
        self.settled_count = 0
        self.published = False
        self.get_logger().info(
            f'Waiting for robot ready (ns={ns}, '
            f'need {self.required_msgs} settled msgs, '
            f'max_tilt={math.degrees(self.max_tilt):.0f}deg, '
            f'max_vz={self.max_vz}m/s)...')

    def joints_cb(self, msg):
        if not self.joints_received and len(msg.position) > 0:
            self.joints_received = True
            self.get_logger().info('Joint states received — controllers active.')

    def state_cb(self, msg):
        if self.published:
            return

        if not self.joints_received:
            return

        roll, pitch = quaternion_to_rp(msg.body.pose.orientation)
        vz = abs(msg.body.twist.linear.z)
        z = msg.body.pose.position.z

        upright = abs(roll) < self.max_tilt and abs(pitch) < self.max_tilt
        landed = vz < self.max_vz and z < 1.0  # must be near ground, not mid-air

        if upright and landed:
            self.settled_count += 1
        else:
            self.settled_count = 0

        if self.settled_count >= self.required_msgs:
            self.published = True
            ready_msg = Bool()
            ready_msg.data = True
            self.ready_pub.publish(ready_msg)
            self.get_logger().info('Robot is ready — upright and settled!')
            raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = WaitForRobotNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
