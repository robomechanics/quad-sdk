#!/usr/bin/env python3
"""
Episode monitor node for training data collection.

Monitors three termination conditions:
  1. Goal reached    — global planner publishes on goal_reached topic
  2. Planner failure — local planner publishes on planner_failed topic
  3. Body collision  — robot orientation (roll/pitch) exceeds threshold

On any condition, waits a brief settle period to flush bag data,
then raises SystemExit to trigger a clean launch shutdown.
"""

import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from quad_msgs.msg import RobotState


def quaternion_to_rpy(q):
    """Convert geometry_msgs Quaternion to (roll, pitch, yaw) in radians."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    return roll, pitch


class EpisodeMonitorNode(Node):
    def __init__(self):
        super().__init__('episode_monitor')

        # -- Parameters ------------------------------------------------------------
        self.declare_parameter('namespace', 'robot_1')
        self.declare_parameter('settle_time', 2.0)
        self.declare_parameter('max_tilt', 1.0)  # radians (~57 deg)

        ns = self.get_parameter('namespace').value
        self.settle_time = self.get_parameter('settle_time').value
        self.max_tilt = self.get_parameter('max_tilt').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # -- Subscriptions ---------------------------------------------------------
        self.create_subscription(
            Bool, f'/{ns}/goal_reached', self.goal_reached_cb, 10)
        self.create_subscription(
            Bool, f'/{ns}/planner_failed', self.planner_failed_cb, 10)
        self.create_subscription(
            RobotState, f'/{ns}/state/ground_truth', self.state_cb, qos)

        # -- State tracking --------------------------------------------------------
        self.shutdown_pending = False

        self.get_logger().info(
            f'Episode monitor started (ns={ns}, '
            f'max_tilt={math.degrees(self.max_tilt):.0f}deg)')

    # -- Callbacks -----------------------------------------------------------------

    def goal_reached_cb(self, msg):
        if msg.data:
            self.request_shutdown('Goal reached')

    def planner_failed_cb(self, msg):
        if msg.data:
            self.request_shutdown('Local planner failure')

    def state_cb(self, msg):
        if self.shutdown_pending:
            return

        roll, pitch = quaternion_to_rpy(msg.body.pose.orientation)

        if abs(roll) > self.max_tilt or abs(pitch) > self.max_tilt:
            self.request_shutdown(
                f'Body collision — tilt (roll={math.degrees(roll):.1f}deg, '
                f'pitch={math.degrees(pitch):.1f}deg) exceeds '
                f'{math.degrees(self.max_tilt):.0f}deg')

    # -- Shutdown ------------------------------------------------------------------

    def request_shutdown(self, reason):
        if self.shutdown_pending:
            return
        self.shutdown_pending = True
        self.get_logger().info(
            f'{reason}! Shutting down in {self.settle_time}s...')
        self.create_timer(self.settle_time, self.trigger_shutdown)

    def trigger_shutdown(self):
        self.get_logger().info('Episode complete, requesting shutdown.')
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = EpisodeMonitorNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
