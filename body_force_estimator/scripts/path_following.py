#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from quad_msgs.msg import RobotState
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener, TransformException

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
        # speed 0.50 matches runs 67/68, the last configuration that walked.
        # Step length is speed * gait period (0.36 s, go2.yaml); cadence is
        # fixed, so speed sets stride length only.
        self.rate_hz = 100
        self.declare_parameter('speed', 0.5)      # Forward velocity
        self.declare_parameter('y_pt', 0.0)        # Lateral setpoint (beam centreline y)
        self.declare_parameter('yaw_pt', 0.0)      # Yaw setpoint (beam heading)
        self.declare_parameter('y_gain', 1.0)      # Lateral P gain
        self.declare_parameter('yaw_gain', 3.0)    # Yaw P gain
        self.declare_parameter('speed_i', 0.0)     # Integration rate

        # Express the beam in the ROBOT frame and steer on that.
        #
        # y_pt/yaw_pt above are constants in the MAP frame, so "straight" means
        # world +x at world y=0 -- which is only the beam's centreline if the
        # beam happens to lie on that axis, and it does not.
        #
        # Looking the beam up through TF as body->beam gives the error directly
        # in the body frame, which is also the frame local_planner interprets
        # cmd_vel in:
        #     world_vx = vx*cos(yaw) - vy*sin(yaw)
        # So a body-frame error needs no rotation, and the long-standing
        # mismatch (world-frame error published as a body-frame velocity, which
        # inverts the lateral correction past ~3 deg of yaw) disappears.
        #
        # Empty beam_frame keeps the original world-frame behaviour.
        self.declare_parameter('beam_frame', '')
        self.declare_parameter('body_frame', 'body')

        self.speed = self.get_parameter('speed').value
        self.y_pt = self.get_parameter('y_pt').value
        self.yaw_pt = self.get_parameter('yaw_pt').value
        self.y_gain = self.get_parameter('y_gain').value
        self.yaw_gain = self.get_parameter('yaw_gain').value
        self.speed_i = self.get_parameter('speed_i').value
        self.beam_frame = self.get_parameter('beam_frame').value
        self.body_frame = self.get_parameter('body_frame').value

        self.tf_buffer_ = None
        if self.beam_frame:
            self.tf_buffer_ = Buffer()
            self.tf_listener_ = TransformListener(self.tf_buffer_, self)
            self.get_logger().info(
                f"Steering on the beam in the ROBOT frame: TF "
                f"'{self.body_frame}' -> '{self.beam_frame}'")
        else:
            self.get_logger().info(
                f"Following the world axis: y_pt={self.y_pt}, yaw_pt={self.yaw_pt}")

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

    def beam_error_body(self):
        """Lateral and heading error to the beam, expressed in the BODY frame.

        Returns (y_err, yaw_err) or None if the transform is unavailable.
        Both are already body-frame, so they can be commanded directly.
        """
        try:
            tf = self.tf_buffer_.lookup_transform(
                self.body_frame, self.beam_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f"no {self.body_frame}->{self.beam_frame} transform: {ex}",
                throttle_duration_sec=5.0)
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        # Heading of the beam relative to the robot.
        yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
        # A symmetric beam looks the same either way round, and the Motive
        # rigid body has been seen flipping 180 deg. Fold into [-pi/2, pi/2]
        # so a flip does not command a U-turn.
        if yaw > math.pi/2:
            yaw -= math.pi
        elif yaw < -math.pi/2:
            yaw += math.pi

        # Perpendicular distance from the robot (body origin) to the beam
        # centreline. The line passes through (t.x, t.y) with direction yaw,
        # both already in body coordinates, so this is a body-frame offset:
        # positive means the beam is to the robot's left.
        y_err = -(-t.x*math.sin(yaw) + t.y*math.cos(yaw))
        return y_err, -yaw

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

        # Errors. Beam mode gives them directly in the body frame; world mode
        # computes them in the map frame as the original did.
        beam = self.beam_error_body() if self.tf_buffer_ else None
        if beam is not None:
            y_err, yaw_error = beam
        else:
            y_err = self.last_state_msg_.body.pose.position.y - self.y_pt
            yaw_error = euler[2] - self.yaw_pt

        if yaw_error > math.pi:
            yaw_error -= 2*math.pi
        elif yaw_error < -math.pi:
            yaw_error += 2*math.pi

        # Velocity commands. cmd_vel is interpreted by local_planner in the
        # BODY frame:
        #     world_vx = vx*cos(yaw) - vy*sin(yaw)
        # In beam mode the errors above are already body-frame, so publishing
        # them directly is correct. In world mode they are map-frame, which is
        # the original 75810edf behaviour and is only valid while yaw ~ 0.
        cmd.linear.x = self.speed + self.speed_integral
        cmd.linear.y = -self.speed_sign * self.y_gain * y_err
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
