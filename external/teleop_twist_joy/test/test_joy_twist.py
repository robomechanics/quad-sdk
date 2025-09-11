# Software License Agreement (BSD)
#
# @author    Tony Baltovski <tbaltovski@clearpathrobotics.com>
# @copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions
#   and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of
#   conditions and the following disclaimer in the documentation and/or other materials provided
#   with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse
#   or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import time
import unittest

import geometry_msgs.msg
import launch_testing_ros
import rclpy
import sensor_msgs.msg


class TestJoyTwist(unittest.TestCase):

    def setUp(self):
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('test_joy_twist_node', context=self.context,
                                      allow_undeclared_parameters=True,
                                      automatically_declare_parameters_from_overrides=True)
        self.message_pump = launch_testing_ros.MessagePump(self.node, context=self.context)
        self.pub = self.node.create_publisher(sensor_msgs.msg.Joy, 'joy', 1)
        if not hasattr(self, 'cmd_vel_msg_type'):
            self.cmd_vel_msg_type = geometry_msgs.msg.Twist
        self.sub = self.node.create_subscription(self.cmd_vel_msg_type,
                                                 'cmd_vel', self.callback, 1)
        self.message_pump.start()

        self.expect_cmd_vel = {
                'angular': {
                    'x': self.node.get_parameter('expect_cmd_vel.angular.x').value or 0.0,
                    'y': self.node.get_parameter('expect_cmd_vel.angular.y').value or 0.0,
                    'z': self.node.get_parameter('expect_cmd_vel.angular.z').value or 0.0,
                },
                'linear': {
                    'x': self.node.get_parameter('expect_cmd_vel.linear.x').value or 0.0,
                    'y': self.node.get_parameter('expect_cmd_vel.linear.y').value or 0.0,
                    'z': self.node.get_parameter('expect_cmd_vel.linear.z').value or 0.0,
                },
            }
        self.joy_msg = {
                'axes': self.node.get_parameter('publish_joy.axes').value or [],
                'buttons': self.node.get_parameter('publish_joy.buttons').value or [],
            }
        self.received_cmd_vel = None

    def tearDown(self):
        self.message_pump.stop()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)

    def test_expected(self):
        pub_joy = sensor_msgs.msg.Joy()
        pub_joy.axes.extend(self.joy_msg['axes'])
        pub_joy.buttons.extend(self.joy_msg['buttons'])
        while self.received_cmd_vel is None:
            self.pub.publish(pub_joy)
            time.sleep(0.1)

        self.assertAlmostEqual(self.received_cmd_vel.linear.x,
                               self.expect_cmd_vel['linear']['x'])
        self.assertAlmostEqual(self.received_cmd_vel.linear.y,
                               self.expect_cmd_vel['linear']['y'])
        self.assertAlmostEqual(self.received_cmd_vel.linear.z,
                               self.expect_cmd_vel['linear']['z'])
        self.assertAlmostEqual(self.received_cmd_vel.angular.x,
                               self.expect_cmd_vel['angular']['x'])
        self.assertAlmostEqual(self.received_cmd_vel.angular.y,
                               self.expect_cmd_vel['angular']['y'])
        self.assertAlmostEqual(self.received_cmd_vel.angular.z,
                               self.expect_cmd_vel['angular']['z'])

    def callback(self, msg):
        self.received_cmd_vel = geometry_msgs.msg.Twist()
        if self.cmd_vel_msg_type is geometry_msgs.msg.Twist:
            self.received_cmd_vel = msg
        else:
            self.received_cmd_vel = msg.twist
