#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from quad_msgs.msg import RobotState
from scipy.spatial.transform import Rotation as R

# World-frame path following for v107+ policies (EpisodeWorldVelocityCommand
# training semantics): hold a WORLD-frame velocity target and rotate it into
# the current body frame every tick, so the policy sees its own yaw drift as
# command rotation and self-corrects — exactly as in training.
#
# Use plain path_following.py for pre-v107 (body-frame trained) policies.
# Rationale (2026-09-20 bags): body-frame P-corrections fight the world-frame
# policy's internal heading correction; vy railed at +-1.0 m/s and wz spiked
# to 6.8 rad/s, destroying the gait. All corrections here are clamped to the
# command ranges seen in training (vy via rotation only, |wz| <= 0.3).

class WorldPathFollowingNode(Node):
    def __init__(self):
        super().__init__('world_path_following')

        self.last_state_msg_ = RobotState()

        # Parameters
        self.declare_parameter('speed', 0.5)     # world +x speed, m/s
        # Publish cmd=0 for the first warmup_secs after the first state msg.
        # The robot_driver stand gate holds PD stance while the policy's GRU
        # integrates real stance observations, so its latent has converged
        # before the walk command arrives (Isaac probe 2026-09-21: 10 s of
        # policy-active stance -> clean walk-off through the cmd step; the
        # cold-start h=0 + immediate 0.5 cmd is when RL gets tucked).
        self.declare_parameter('warmup_secs', 10.0)
        self.declare_parameter('y_pt', 0.0)      # target line y, m
        self.declare_parameter('y_gain', 0.3)    # world cross-track P gain
        self.declare_parameter('vy_max', 0.15)   # clamp on cross-track term, m/s
        self.declare_parameter('yaw_gain', 0.5)  # yaw-hold P gain
        self.declare_parameter('wz_max', 0.3)    # clamp on wz; training used +-0.3
        self.speed = float(self.get_parameter('speed').value)
        self.warmup_secs = float(self.get_parameter('warmup_secs').value)
        self.y_pt = float(self.get_parameter('y_pt').value)
        self.y_gain = float(self.get_parameter('y_gain').value)
        self.vy_max = float(self.get_parameter('vy_max').value)
        self.yaw_gain = float(self.get_parameter('yaw_gain').value)
        self.wz_max = float(self.get_parameter('wz_max').value)
        self.yaw_pt = 0.0
        self.rate_hz = 100

        self.pub_ = self.create_publisher(Twist, '/robot_1/cmd_vel', 10)
        self.sub_ = self.create_subscription(
            RobotState, '/robot_1/state/ground_truth', self.state_callback, 10)
        self.timer_ = self.create_timer(1.0 / self.rate_hz, self.control_loop)
        self.state_received_ = False
        self.first_state_time_ = None

    def state_callback(self, msg: RobotState):
        self.last_state_msg_ = msg
        self.state_received_ = True

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def control_loop(self):
        if not self.state_received_:
            self.get_logger().info('world_path_following: no state message',
                                   throttle_duration_sec=2.0)
            return

        # Stance warmup: hold cmd=0 so the stand gate keeps the robot in PD
        # stance while the GRU latent converges on real observations.
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.first_state_time_ is None:
            self.first_state_time_ = now
        if now - self.first_state_time_ < self.warmup_secs:
            self.pub_.publish(Twist())
            self.get_logger().info(
                f'warmup: holding cmd=0 '
                f'({self.warmup_secs - (now - self.first_state_time_):.0f} s left)',
                throttle_duration_sec=2.0)
            return

        q = self.last_state_msg_.body.pose.orientation
        yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
        yaw_error = self._wrap(yaw - self.yaw_pt)
        y_err = self.last_state_msg_.body.pose.position.y - self.y_pt

        # World-frame velocity target: forward along +x plus a small, clamped
        # cross-track term to hold the line over long runs.
        v_wx = self.speed
        v_wy = self._clamp(-self.y_gain * y_err, -self.vy_max, self.vy_max)

        # Rotate the world vector into the body frame — the deploy-time
        # equivalent of EpisodeWorldVelocityCommand's per-step refresh.
        c, s = math.cos(yaw), math.sin(yaw)
        cmd = Twist()
        cmd.linear.x = c * v_wx + s * v_wy
        cmd.linear.y = -s * v_wx + c * v_wy

        # Gentle, clamped yaw hold: the policy keeps its velocity DIRECTION
        # correct on its own; this only keeps the body pointed down-track.
        cmd.angular.z = self._clamp(-self.yaw_gain * yaw_error,
                                    -self.wz_max, self.wz_max)

        self.pub_.publish(cmd)
        self.get_logger().info(
            f'cmd vx={cmd.linear.x:.2f} vy={cmd.linear.y:.2f} '
            f'wz={cmd.angular.z:.2f} (yaw={math.degrees(yaw):.0f} deg, '
            f'y_err={y_err:.2f} m)',
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = WorldPathFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
