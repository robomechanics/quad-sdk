#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from quad_msgs.msg import RobotState
from scipy.spatial.transform import Rotation as R

# Command the robot to walk along a straight line parallel to the x-axis with
# zero yaw

class PathFollowingNode(Node):
    def __init__(self):
        super().__init__('path_following')

        # Robot state message
        self.last_state_msg_ = RobotState()

        # Parameters
        self.rate_hz = 100
        self.speed = 0.5   # Forward velocity
        self.y_pt = 0.0    # Lateral shift
        self.yaw_pt = 0.0  # Yaw angle
        self.y_gain = 1.0  # Lateral P gain
        self.yaw_gain = 3.0
        self.speed_i = 0.0 # Integration rate

        self.speed_integral = 0.0
        self.speed_sign = (self.speed > 0)*1 + (self.speed < 0)*-1

        # Publisher
        self.pub_ = self.create_publisher(
            Twist,
            '/robot_1/cmd_vel',
            10
        )

        # Subscriber
        self.sub_ = self.create_subscription(
            RobotState,
            '/robot_1/state/ground_truth',
            self.state_callback,
            10
        )

        # Timer
        self.timer_ = self.create_timer(1.0/self.rate_hz, self.control_loop)

        self.state_received_ = False

    def state_callback(self, msg: RobotState):
        self.last_state_msg_ = msg
        self.state_received_ = True

    def control_loop(self):
        if not self.state_received_:
            self.get_logger().info('path_following: no state message')
            return

        cmd = Twist()
        q = self.last_state_msg_.body.pose.orientation
        euler = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=False)

        # x velocity error integral
        speed_error = self.last_state_msg_.body.twist.linear.x - self.speed
        if speed_error*self.speed_sign < 0:
            self.speed_integral -= self.speed_i/self.rate_hz * speed_error
        else:
            self.speed_integral -= self.speed_i/self.rate_hz * speed_error * 10

        # Yaw angle
        yaw_error = euler[2] - self.yaw_pt
        if yaw_error > math.pi:
            yaw_error -= 2*math.pi
        elif yaw_error < -math.pi:
            yaw_error += 2*math.pi

        # Velocity commands
        cmd.linear.x = self.speed + self.speed_integral
        cmd.linear.y = -self.speed_sign * self.y_gain * (self.last_state_msg_.body.pose.position.y - self.y_pt)
        cmd.angular.z = -self.yaw_gain * yaw_error

        self.pub_.publish(cmd)
        self.get_logger().info(f"Publishing cmd: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
