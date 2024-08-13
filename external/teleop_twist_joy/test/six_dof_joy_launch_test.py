import launch
import launch_ros.actions
import launch_testing

import pytest

import test_joy_twist


@pytest.mark.rostest
def generate_test_description():
    teleop_node = launch_ros.actions.Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        parameters=[{
            'axis_linear.x': 1,
            'axis_linear.y': 2,
            'axis_linear.z': 3,
            'axis_angular.roll': 4,
            'axis_angular.pitch': 5,
            'axis_angular.yaw': 0,
            'scale_linear.x': 1.0,
            'scale_linear.y': 4.0,
            'scale_linear.z': 2.0,
            'scale_angular.roll': 3.0,
            'scale_angular.pitch': 2.0,
            'scale_angular.yaw': 1.0,
            'enable_button': 0,
        }],
    )

    return launch.LaunchDescription([
            teleop_node,
            launch_testing.actions.ReadyToTest(),
        ]), locals()


class SixDofJoy(test_joy_twist.TestJoyTwist):

    def setUp(self):
        super().setUp()
        self.joy_msg['axes'] = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
        self.joy_msg['buttons'] = [1]
        self.expect_cmd_vel['linear']['x'] = 0.4
        self.expect_cmd_vel['linear']['y'] = 2.0
        self.expect_cmd_vel['linear']['z'] = 1.2
        self.expect_cmd_vel['angular']['x'] = 2.1
        self.expect_cmd_vel['angular']['y'] = 1.6
        self.expect_cmd_vel['angular']['z'] = 0.3
