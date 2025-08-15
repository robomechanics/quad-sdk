import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from quad_msgs.msg import RobotState


class LegFlailNode(Node):
    def __init__(self):
        super().__init__('leg_flail')

        self.last_state_msg_ = RobotState()

        self.pub_ = self.create_publisher(
            Vector3,
            '/robot_1/control/single_joint_command',
            10
        )
        # self.sub_ = self.create_subscription(
        #     RobotState,
        #     '/robot_1/state/ground_truth',
        #     self.callback,
        #     10
        # )

        self.timer_ = self.create_timer(0.01, self.timer_callback)  # 100 Hz

        self.i = 0
        self.j = 0

    def callback(self, msg: RobotState):
        self.last_state_msg_ = msg

    def timer_callback(self):
        cmd = Vector3()
        cmd.x = float(self.i)
        cmd.y = float(self.j)
        cmd.z = 10.0

        self.pub_.publish(cmd)

        # Increment i, j in same nested loop style
        self.j += 1
        if self.j >= 3:
            self.j = 0
            self.i += 1
            if self.i >= 4:
                self.i = 0


def main(args=None):
    rclpy.init(args=args)
    node = LegFlailNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()