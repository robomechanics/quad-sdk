#!/usr/bin/env python
import rospy
from spirit_msgs.msg import LegCommand
import numpy as np
import odrive
from odrive.enums import *
from odrive.utils import *
import time
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import TwistStamped


print("finding an odrive...")
odrv0_ = odrive.find_any()
print("found an odrive")
while odrv0_.axis0.current_state != AXIS_STATE_IDLE or \
        odrv0_.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
dump_errors(odrv0_, True)

# Set up the motors!
odrv0_.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0_.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Pitch and roll start point
start_pitch_ = odrv0_.axis0.encoder.pos_estimate
start_roll_ = odrv0_.axis1.encoder.pos_estimate

emergency_stop_ = False

update_rate_ = rospy.get_param("/tail_controller/odrive_update_rate")
gear_ratio_ = rospy.get_param("/tail_controller/gear_ratio")

tail_control_topic_ = rospy.get_param("/topics/control/tail_command")
odrive_encoder_topic_ = rospy.get_param("/topics/odrive_encoder")

vel_topic_ = rospy.get_param("/topics/vel")
imu_topic_ = rospy.get_param("/topics/imu")

tail_control_sub_ = rospy.Subscriber(
    tail_control_topic_, LegCommand, tailControlCallback, queue_size=1)
odrive_encoder_pub_ = rospy.Publisher(
    odrive_encoder_topic_, JointState, queue_size=1)

vel_sub_ = rospy.Subscriber(
    vel_topic_, TwistStamped, velCallback, queue_size=1)
imu_sub_ = rospy.Subscriber(
    imu_topic_, Imu, imuCallback, queue_size=1)

tail_control_msg_ = None

vel_msg_ = None
imu_msg_ = None


def tailControlCallback(msg):
    global tail_control_msg_
    tail_control_msg_ = msg


def publish_encoder():
    pos_roll = (odrv0_.axis1.encoder.pos_estimate -
                start_roll_)*np.pi*2/gear_ratio_
    pos_pitch = (odrv0_.axis0.encoder.pos_estimate -
                 start_pitch_)*np.pi*2/gear_ratio_
    vel_roll = (odrv0_.axis1.encoder.vel_estimate)*np.pi*2/gear_ratio_
    vel_pitch = (odrv0_.axis0.encoder.vel_estimate)*np.pi*2/gear_ratio_

    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.position.append(pos_roll)
    msg.position.append(pos_pitch)
    msg.velocity.append(vel_roll)
    msg.velocity.append(vel_pitch)
    msg.effort.append(0)
    msg.effort.append(0)
    odrive_encoder_pub_.publish(msg)


def motor_control():
    pos_roll = (odrv0_.axis1.encoder.pos_estimate -
                start_roll_)*np.pi*2/gear_ratio_
    pos_pitch = (odrv0_.axis0.encoder.pos_estimate -
                 start_pitch_)*np.pi*2/gear_ratio_
    vel_roll = (odrv0_.axis1.encoder.vel_estimate)*np.pi*2/gear_ratio_
    vel_pitch = (odrv0_.axis0.encoder.vel_estimate)*np.pi*2/gear_ratio_

    if emergency_stop_ or \
            np.abs(pos_roll) > np.pi*0.5 or \
            np.abs(pos_pitch) > np.pi*0.5:
        odrv0_.axis0.controller.input_pos = 0
        odrv0_.axis1.controller.input_pos = 0

        odrv0_.axis0.controller.input_vel = 0
        odrv0_.axis1.controller.input_vel = 0

        odrv0_.axis0.controller.input_torque = 0
        odrv0_.axis1.controller.input_torque = 0

        odrv0_.axis0.requested_state = AXIS_STATE_IDLE
        odrv0_.axis1.requested_state = AXIS_STATE_IDLE

        emergency_stop_ = True

    else:
        if (not imu_msg_) or (not vel_msg_):
            odrv0_.axis0.controller.input_pos = 0
            odrv0_.axis1.controller.input_pos = 0

            odrv0_.axis0.controller.input_vel = 0
            odrv0_.axis1.controller.input_vel = 0

            odrv0_.axis0.controller.input_torque = 0
            odrv0_.axis1.controller.input_torque = 0
        else:
            # roll
            sinr_cosp = 2 * (imu_msg_.orientation.w * imu_msg_.orientation.x +
                             imu_msg_.orientation.y * imu_msg_.orientation.z)
            cosr_cosp = 1 - 2 * (imu_msg_.orientation.x * imu_msg_.orientation.x +
                                 imu_msg_.orientation.y * imu_msg_.orientation.y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)

            # pitch
            sinp = 2 * (imu_msg_.orientation.w * imu_msg_.orientation.y -
                        imu_msg_.orientation.z * imu_msg_.orientation.x)
            if np.abs(sinp) >= 1:
                pitch = np.sign(sinp)*np.pi/2
            else:
                pitch = np.arcsin(sinp)

            # yaw
            siny_cosp = 2 * (imu_msg_.orientation.w * imu_msg_.orientation.z +
                             imu_msg_.orientation.x * imu_msg_.orientation.y)
            cosy_cosp = 1 - 2 * (imu_msg_.orientation.y * imu_msg_.orientation.y +
                                 imu_msg_.orientation.z * imu_msg_.orientation.z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

            wx = vel_msg_.twist.angular.x
            wy = vel_msg_.twist.angular.y
            wz = vel_msg_.twist.angular.z

            odrv0_.axis1.controller.input_pos = start_roll_ + \
                gear_ratio_ / (2*np.pi)*roll
            odrv0_.axis1.controller.input_vel = gear_ratio_ / (2*np.pi)*wx
            odrv0_.axis1.controller.input_torque = 0

            odrv0_.axis0.controller.input_pos = start_pitch_ + \
                gear_ratio_ / (2*np.pi)*pitch
            odrv0_.axis0.controller.input_vel = gear_ratio_ / (2*np.pi)*wy
            odrv0_.axis0.controller.input_torque = 0

        # if not tail_control_msg_:
        #     odrv0_.axis0.controller.input_pos = 0
        #     odrv0_.axis1.controller.input_pos = 0

        #     odrv0_.axis0.controller.input_vel = 0
        #     odrv0_.axis1.controller.input_vel = 0

        #     odrv0_.axis0.controller.input_torque = 0
        #     odrv0_.axis1.controller.input_torque = 0
        # else:
        #     odrv0_.axis1.controller.config.pos_gain = tail_control_msg_.motor_commands[0].kp
        #     odrv0_.axis1.controller.config.vel_gain = tail_control_msg_.motor_commands[0].kd
        #     odrv0_.axis1.controller.input_pos = start_roll_ + gear_ratio_ / \
        #         (2*np.pi)*tail_control_msg_.motor_commands[0].pos_setpoint
        #     odrv0_.axis1.controller.input_vel = gear_ratio_ / \
        #         (2*np.pi)*tail_control_msg_.motor_commands[0].vel_setpoint
        #     odrv0_.axis1.controller.input_torque = tail_control_msg_.motor_commands[
        #         0].torque_ff/gear_ratio_

        #     odrv0_.axis0.controller.config.pos_gain = tail_control_msg_.motor_commands[1].kp
        #     odrv0_.axis0.controller.config.vel_gain = tail_control_msg_.motor_commands[1].kd
        #     odrv0_.axis0.controller.input_pos = start_pitch_ + gear_ratio_ / \
        #         (2*np.pi)*tail_control_msg_.motor_commands[1].pos_setpoint
        #     odrv0_.axis0.controller.input_vel = gear_ratio_ / \
        #         (2*np.pi)*tail_control_msg_.motor_commands[1].vel_setpoint
        #     odrv0_.axis0.controller.input_torque = tail_control_msg_.motor_commands[
        #         1].torque_ff/gear_ratio_


def imuCallback(msg):
    global imu_msg_
    imu_msg_ = msg


def velCallback(msg):
    global vel_msg_
    vel_msg_ = msg


def spin():
    global odrv0_

    rospy.init_node('odrive_converter', anonymous=True)

    r = rospy.Rate(update_rate_)
    while not rospy.is_shutdown():
        publish_encoder()
        motor_control()
        r.sleep()

    # put motors into idle when program is killed
    odrv0_.axis0.requested_state = AXIS_STATE_IDLE
    odrv0_.axis1.requested_state = AXIS_STATE_IDLE


if __name__ == '__main__':
    spin()
