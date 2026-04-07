#!/usr/bin/env python3
"""
Time-synchronized republisher for training data collection.

Subscribes to key simulation topics running at different rates, aligns them
using ApproximateTimeSynchronizer, and republishes the synchronized tuples
on /synced/<topic> so they can be bagged as coherent (state, action) pairs.

Synchronized topics:
  - state/ground_truth    (RobotState,       ~500 Hz)  robot state observation
  - control/joint_command (LegCommandArray,  ~500 Hz)  joint-level actions
  - control/grfs          (GRFArray,         ~500 Hz)  ground reaction forces
  - local_plan            (RobotPlan,        ~333 Hz)  MPC local plan
  - state/estimate        (RobotState,       ~500 Hz)  state estimate

The node publishes at the rate of the slowest topic in each synchronized
bundle (bounded by the slop tolerance).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from message_filters import Subscriber, ApproximateTimeSynchronizer

from quad_msgs.msg import RobotState, LegCommandArray, GRFArray, RobotPlan


class TimeSyncNode(Node):
    def __init__(self):
        super().__init__('time_sync')

        # -- Parameters ------------------------------------------------------------
        self.declare_parameter('slop', 0.02)
        self.declare_parameter('queue_size', 30)
        self.declare_parameter('namespace', 'robot_1')

        slop = self.get_parameter('slop').value
        queue_size = self.get_parameter('queue_size').value
        ns = self.get_parameter('namespace').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
        )

        # -- Subscribers (via message_filters) -------------------------------------
        self.sub_state = Subscriber(
            self, RobotState, f'/{ns}/state/ground_truth', qos_profile=qos)
        self.sub_cmd = Subscriber(
            self, LegCommandArray, f'/{ns}/control/joint_command', qos_profile=qos)
        self.sub_grfs = Subscriber(
            self, GRFArray, f'/{ns}/control/grfs', qos_profile=qos)
        self.sub_plan = Subscriber(
            self, RobotPlan, f'/{ns}/local_plan', qos_profile=qos)
        self.sub_est = Subscriber(
            self, RobotState, f'/{ns}/state/estimate', qos_profile=qos)

        # -- Synchronizer ----------------------------------------------------------
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_state, self.sub_cmd, self.sub_grfs,
             self.sub_plan, self.sub_est],
            queue_size=queue_size,
            slop=slop,
        )
        self.sync.registerCallback(self.synced_callback)

        # -- Publishers (synced topics) --------------------------------------------
        self.pub_state = self.create_publisher(
            RobotState, f'/{ns}/synced/state/ground_truth', 10)
        self.pub_cmd = self.create_publisher(
            LegCommandArray, f'/{ns}/synced/control/joint_command', 10)
        self.pub_grfs = self.create_publisher(
            GRFArray, f'/{ns}/synced/control/grfs', 10)
        self.pub_plan = self.create_publisher(
            RobotPlan, f'/{ns}/synced/local_plan', 10)
        self.pub_est = self.create_publisher(
            RobotState, f'/{ns}/synced/state/estimate', 10)

        self.sync_count = 0
        self.get_logger().info(
            f'Time sync node started (ns={ns}, slop={slop}s, queue={queue_size})')

    def synced_callback(self, state_msg, cmd_msg, grfs_msg, plan_msg, est_msg):
        """Republish the time-aligned bundle."""
        self.pub_state.publish(state_msg)
        self.pub_cmd.publish(cmd_msg)
        self.pub_grfs.publish(grfs_msg)
        self.pub_plan.publish(plan_msg)
        self.pub_est.publish(est_msg)

        self.sync_count += 1
        if self.sync_count % 1000 == 0:
            self.get_logger().info(f'Synced {self.sync_count} bundles')


def main(args=None):
    rclpy.init(args=args)
    node = TimeSyncNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
