#!/usr/bin/env python
# Software License Agreement (BSD)
#
# @author    Tony Baltovski <tbaltovski@clearpathrobotics.com>
# @copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import unittest
import time
import rostest
import rospy
import geometry_msgs.msg
import sensor_msgs.msg


class TestJoyTwist(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_joy_twist_node', anonymous=True)
        self.pub = rospy.Publisher('joy', sensor_msgs.msg.Joy)
        rospy.Subscriber('cmd_vel', geometry_msgs.msg.Twist, callback=self.callback)

        while (not rospy.has_param("~publish_joy")) and (not rospy.get_param("~expect_cmd_vel")):
            time.sleep(0.1)

        self.expect_cmd_vel = rospy.get_param("~expect_cmd_vel")
        self.joy_msg = rospy.get_param("~publish_joy")
        self.received_cmd_vel = None

    def test_expected(self):
        pub_joy = sensor_msgs.msg.Joy()
        pub_joy.axes.extend(self.joy_msg['axes'])
        pub_joy.buttons.extend(self.joy_msg['buttons'])
        while self.received_cmd_vel is None:
            self.pub.publish(pub_joy)
            time.sleep(0.1)

        self.assertAlmostEqual(self.received_cmd_vel.linear.x, self.expect_cmd_vel['linear']['x'])
        self.assertAlmostEqual(self.received_cmd_vel.linear.y, self.expect_cmd_vel['linear']['y'])
        self.assertAlmostEqual(self.received_cmd_vel.linear.z, self.expect_cmd_vel['linear']['z'])
        self.assertAlmostEqual(self.received_cmd_vel.angular.x, self.expect_cmd_vel['angular']['x'])
        self.assertAlmostEqual(self.received_cmd_vel.angular.y, self.expect_cmd_vel['angular']['y'])
        self.assertAlmostEqual(self.received_cmd_vel.angular.z, self.expect_cmd_vel['angular']['z'])

    def callback(self, msg):
        self.received_cmd_vel = geometry_msgs.msg.Twist()
        self.received_cmd_vel = msg


if __name__ == '__main__':
    rostest.rosrun('teleop_twist_joy', 'test_joy_twist', TestJoyTwist)
