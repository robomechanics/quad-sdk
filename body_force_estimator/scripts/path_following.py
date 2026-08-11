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

        # Parameters. Exposed as ROS params so beam runs can be retuned without
        # a rebuild.
        #
        # Step length is speed * gait period, and period is fixed at 0.36 s
        # (go2.yaml), so the cadence does not change with speed -- only the
        # stride does. At 0.25 the robot tracked ~0.16 m/s, giving ~5.8 cm
        # steps: stable but a visible shuffle. 0.40 restores roughly 9 cm
        # strides. The two failures that motivated slowing down (the cmd_vel
        # frame error, and the corridor being wide enough that footholds were
        # planned onto the beam edge) are both fixed, so the earlier 0.5 is
        # likely viable again -- this is a live ROS param, so raise it there
        # rather than editing here.
        self.rate_hz = 100
        self.declare_parameter('speed', 0.40)      # Forward velocity
        self.declare_parameter('y_pt', 0.0)        # Lateral setpoint (beam centreline y)
        self.declare_parameter('yaw_pt', 0.0)      # Yaw setpoint (beam heading)
        self.declare_parameter('y_gain', 1.0)      # Lateral P gain
        self.declare_parameter('yaw_gain', 3.0)    # Yaw P gain
        self.declare_parameter('speed_i', 0.0)     # Integration rate
        # Cap on how far the lateral-error steering may swing the heading
        # setpoint (rad). Keeps a large lateral error from demanding a turn the
        # robot has no authority to make with near-collinear feet on a beam.
        self.declare_parameter('max_heading_offset', 0.30)

        self.speed = self.get_parameter('speed').value
        self.y_pt = self.get_parameter('y_pt').value
        self.yaw_pt = self.get_parameter('yaw_pt').value
        self.y_gain = self.get_parameter('y_gain').value
        self.yaw_gain = self.get_parameter('yaw_gain').value
        self.speed_i = self.get_parameter('speed_i').value
        self.max_heading_offset = self.get_parameter('max_heading_offset').value

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

        # Unicycle steering: the lateral offset from the reference line is
        # folded into a HEADING setpoint rather than commanded as a body-lateral
        # velocity. On a beam narrower than the robot's stance, a lateral
        # velocity command asks the feet to step sideways off the only
        # traversable strip, which they cannot do; steering instead points the
        # robot back at the line and lets forward motion close the error.
        #
        # The heading offset is the direction the old lateral velocity term
        # would have produced, atan2(y_gain * y_err, speed), so y_gain keeps its
        # previous meaning and authority.
        y_err = self.last_state_msg_.body.pose.position.y - self.y_pt
        heading_offset = math.atan2(self.y_gain * y_err,
                                    max(abs(self.speed), 1e-3))
        heading_offset = max(-self.max_heading_offset,
                             min(self.max_heading_offset, heading_offset))

        # y_err > 0 (left of the line) -> steer right -> negative heading.
        yaw_des = self.yaw_pt - self.speed_sign * heading_offset

        yaw_error = euler[2] - yaw_des
        if yaw_error > math.pi:
            yaw_error -= 2*math.pi
        elif yaw_error < -math.pi:
            yaw_error += 2*math.pi

        # Drive straight ahead in the body frame and steer. No lateral velocity
        # is commanded at all, so nothing ever asks the feet to leave the
        # traversable strip sideways.
        cmd.linear.x = self.speed + self.speed_integral
        cmd.linear.y = 0.0
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
