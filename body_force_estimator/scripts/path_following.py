#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from quad_msgs.msg import RobotState
from scipy.spatial.transform import Rotation as R

# Command the robot to walk in a straight line from wherever it is set down.
# The heading and position at the first state message are captured as the
# setpoints: the robot holds its initial facing and the line through its start
# point, and moves along one of its own body axes chosen by `direction`
# (forward / backward / left / right). The policy clips commands to
# |vx| <= 1.0 and |vy| <= 0.5 m/s (underbrush_policy.cpp).

class PathFollowingNode(Node):
    def __init__(self):
        super().__init__('path_following')

        # Robot state message
        self.last_state_msg_ = RobotState()

        # Parameters
        self.rate_hz = 100
        self.direction = 'forward'  # forward | backward | left | right
        self.speed = 0.5   # Speed magnitude along the chosen direction, m/s
        self.line_gain = 1.0  # P gain holding the line through the start point
        # Body-frame unit vector for each direction
        self.axis = {'forward': (1.0, 0.0), 'backward': (-1.0, 0.0),
                     'left': (0.0, 1.0), 'right': (0.0, -1.0)}[self.direction]
        # Captured from the first state message
        self.x0 = 0.0
        self.y0 = 0.0
        self.yaw_pt = 0.0
        self.yaw_gain = 3.0
        self.speed_i = 0.0 # Integration rate

        self.speed_integral = 0.0
        self.speed_sign = 1

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
        if not self.state_received_:
            q = msg.body.pose.orientation
            self.yaw_pt = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
            self.x0 = msg.body.pose.position.x
            self.y0 = msg.body.pose.position.y
            self.get_logger().info(
                f"path_following: start yaw {math.degrees(self.yaw_pt):.1f} deg at "
                f"({self.x0:.2f}, {self.y0:.2f}); walking {self.direction} at {self.speed} m/s")
        self.state_received_ = True

    def control_loop(self):
        if not self.state_received_:
            self.get_logger().info('path_following: no state message')
            return

        cmd = Twist()
        q = self.last_state_msg_.body.pose.orientation
        euler = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=False)

        # Speed error integral along the travel axis (body frame)
        vx_w = self.last_state_msg_.body.twist.linear.x
        vy_w = self.last_state_msg_.body.twist.linear.y
        v_along = (self.axis[0] * (math.cos(euler[2]) * vx_w + math.sin(euler[2]) * vy_w)
                   + self.axis[1] * (-math.sin(euler[2]) * vx_w + math.cos(euler[2]) * vy_w))
        speed_error = v_along - self.speed
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
        # Displacement from the start point, rotated into the start frame
        # (fx along the start heading, fy to its left). The component across
        # the travel axis is the line error; it is corrected on the other axis.
        dx = self.last_state_msg_.body.pose.position.x - self.x0
        dy = self.last_state_msg_.body.pose.position.y - self.y0
        fx = math.cos(self.yaw_pt) * dx + math.sin(self.yaw_pt) * dy
        fy = -math.sin(self.yaw_pt) * dx + math.cos(self.yaw_pt) * dy
        v = self.speed + self.speed_integral
        if self.axis[0] != 0.0:   # forward/backward: hold fy with linear.y
            cmd.linear.x = self.axis[0] * v
            cmd.linear.y = -self.line_gain * fy
        else:                     # left/right: hold fx with linear.x
            cmd.linear.x = -self.line_gain * fx
            cmd.linear.y = self.axis[1] * v
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
