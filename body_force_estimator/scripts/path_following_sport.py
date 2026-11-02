#!/usr/bin/env python3

import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from quad_msgs.msg import RobotState
from std_msgs.msg import UInt8
from scipy.spatial.transform import Rotation as R

# path_following.py for the Go2 STOCK controller (Unitree sport mode).
#
# Same straight-line law as path_following.py (walk along y = y_pt at yaw 0),
# but it feeds go2_sport_bridge_node instead of robot_driver:
#   * state from /robot_1/state/ground_truth, as in path_following.py;
#   * it publishes control/mode = 1 (stand) once at start, which arms the
#     bridge's Move, then /robot_1/cmd_vel at rate_hz;
#   * it stops itself after `duration` s (cmd_vel 0, then silence: the bridge
#     sends StopMove once cmd_vel is older than its cmd_timeout, 0.5 s).
# Ctrl-C does the same.
#
#   ros2 run body_force_estimator path_following_sport.py \
#       --ros-args -p speed:=0.5 -p duration:=10.0

STAND = 1  # control/mode value, as in robot_driver.h


class PathFollowingSportNode(Node):
    def __init__(self):
        super().__init__('path_following_sport')

        self.declare_parameter('speed', 0.5)       # forward velocity, m/s
        self.declare_parameter('duration', 10.0)   # walk time, s
        self.declare_parameter('y_pt', 0.0)        # target line y, m
        self.declare_parameter('yaw_pt', 0.0)      # target yaw, rad
        self.declare_parameter('y_gain', 1.0)      # lateral P gain
        self.declare_parameter('yaw_gain', 3.0)    # yaw P gain
        self.declare_parameter('arm', True)        # send control/mode=1 at start
        self.declare_parameter('namespace', '/robot_1')
        self.speed = float(self.get_parameter('speed').value)
        self.duration = float(self.get_parameter('duration').value)
        self.y_pt = float(self.get_parameter('y_pt').value)
        self.yaw_pt = float(self.get_parameter('yaw_pt').value)
        self.y_gain = float(self.get_parameter('y_gain').value)
        self.yaw_gain = float(self.get_parameter('yaw_gain').value)
        self.arm = bool(self.get_parameter('arm').value)
        ns = self.get_parameter('namespace').value.rstrip('/')

        self.rate_hz = 100
        self.speed_sign = (self.speed > 0)*1 + (self.speed < 0)*-1

        self.pub_ = self.create_publisher(Twist, f'{ns}/cmd_vel', 10)
        self.mode_pub_ = self.create_publisher(UInt8, f'{ns}/control/mode', 1)
        self.sub_ = self.create_subscription(
            RobotState, f'{ns}/state/ground_truth', self.state_callback, 10)
        self.timer_ = self.create_timer(1.0/self.rate_hz, self.control_loop)

        self.last_pose_ = None
        self.last_pose_time_ = None
        self.start_time_ = None
        self.done_ = False

    def state_callback(self, msg: RobotState):
        self.last_pose_ = msg.body.pose
        self.last_pose_time_ = self.get_clock().now()

    def stop(self):
        if self.context.ok():
            self.pub_.publish(Twist())

    def control_loop(self):
        now = self.get_clock().now()
        if self.last_pose_ is None:
            self.get_logger().info('path_following_sport: no state message',
                                   throttle_duration_sec=1.0)
            return
        if (now - self.last_pose_time_).nanoseconds * 1e-9 > 0.2:
            # State lost: command zero; the bridge keeps the robot standing.
            self.stop()
            self.get_logger().warn('path_following_sport: state stale, cmd 0',
                                   throttle_duration_sec=1.0)
            return

        if self.start_time_ is None:
            if self.arm:
                self.mode_pub_.publish(UInt8(data=STAND))
                self.get_logger().info('control/mode = 1 sent (bridge armed)')
            self.start_time_ = now
            self.get_logger().info(
                f'walking {self.speed} m/s for {self.duration} s')
        if (now - self.start_time_).nanoseconds * 1e-9 > self.duration:
            self.stop()
            if not self.done_:
                self.get_logger().info('duration reached, cmd 0 -- Ctrl-C to exit')
                self.done_ = True
            return

        q = self.last_pose_.orientation
        euler = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=False)

        # Yaw angle
        yaw_error = euler[2] - self.yaw_pt
        if yaw_error > math.pi:
            yaw_error -= 2*math.pi
        elif yaw_error < -math.pi:
            yaw_error += 2*math.pi

        # Velocity commands
        cmd = Twist()
        cmd.linear.x = self.speed
        cmd.linear.y = -self.speed_sign * self.y_gain * (self.last_pose_.position.y - self.y_pt)
        cmd.angular.z = -self.yaw_gain * yaw_error
        self.pub_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowingSportNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.stop()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
