#!/usr/bin/env python
import rospy
from spirit_msgs.msg import LegCommand, RobotState
import numpy as np
import odrive
from odrive.enums import *
from odrive.utils import *
import time
from sensor_msgs.msg import Imu

print("finding an odrive...")
odrv0 = odrive.find_any()
print("found an odrive")

while odrv0.axis0.current_state != AXIS_STATE_IDLE or \
        odrv0.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

dump_errors(odrv0, True)

# Pitch start point
start0 = odrv0.axis0.encoder.pos_estimate
# Roll start point
start1 = odrv0.axis1.encoder.pos_estimate

emergency_stop = False


def balance_controller(orientation, angular_velocity):
    if emergency_stop:
        if abs(odrv0.axis0.encoder.vel_estimate)*50/(2*np.pi) < np.deg2rad(0.1) and \
                abs(odrv0.axis1.encoder.vel_estimate)*50/(2*np.pi) < np.deg2rad(0.1):
            odrv0.axis0.requested_state = AXIS_STATE_IDLE
            odrv0.axis1.requested_state = AXIS_STATE_IDLE

    elif abs(orientation[0]) > np.pi/4 or \
            abs(orientation[1]) > np.pi/4 or \
    abs(odrv0.axis0.encoder.pos_estimate-start0)*np.pi*2/50 > np.pi/4 or \
            abs(odrv0.axis1.encoder.pos_estimate-start1)*np.pi*2/50 > np.pi/4:
        emergency_stop = True
        odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        odrv0.axis1.controller.input_pos = 0
        odrv0.axis0.controller.input_pos = 0
        odrv0.axis0.controller.input_vel = 0
        odrv0.axis1.controller.input_vel = 0
        odrv0.axis0.controller.input_torque = 0
        odrv0.axis1.controller.input_torque = 0
        
    else:
        kp, kd = 5, 1

        # gear ratio is 50
        # Position counts in turns
        odrv0.axis1.controller.input_pos = start1 + \
            50/(2*np.pi)*orientation[0]*kp
        odrv0.axis0.controller.input_pos = start0 + \
            50/(2*np.pi)*orientation[0]*kp

        # Velocity counts in turns/sec
        odrv0.axis0.controller.input_vel = angular_velocity[0]*50/(2*np.pi)*kd
        odrv0.axis1.controller.input_vel = angular_velocity[1]*50/(2*np.pi)*kd

        # Torque counts in Nm
        # odrv0.axis0.controller.input_torque = torque_ff[0]/50
        # odrv0.axis1.controller.input_torque = torque_ff[1]/50


def feedback_callback(msg):
    # roll
    sinr_cosp = 2 * (msg.orientation.w * msg.orientation.x +
                     msg.orientation.y * msg.orientation.z)
    cosr_cosp = 1 - 2 * (msg.orientation.x * msg.orientation.x +
                         msg.orientation.y * msg.orientation.y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch
    sinp = 2 * (msg.orientation.w * msg.orientation.y -
                msg.orientation.z * msg.orientation.x)
    if np.abs(sinp) >= 1:
        pitch = np.sign(sinp)*np.pi/2
    else:
        pitch = np.arcsin(sinp)

    # yaw
    siny_cosp = 2 * (msg.orientation.w * msg.orientation.z +
                     msg.orientation.x * msg.orientation.y)
    cosy_cosp = 1 - 2 * (msg.orientation.y * msg.orientation.y +
                         msg.orientation.z * msg.orientation.z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    orientation = [roll, pitch, yaw]
    # print("----------------orientation----------------")
    # print(orientation)

    angular_velocity = [msg.angular_velocity.x,
                        msg.angular_velocity.y, msg.angular_velocity.z]
    # print("----------------angular_velocity----------------")
    # print(angular_velocity)

    balance_controller(orientation, angular_velocity)


def mpc_callback(msg):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.motor_commands[1].pos_setpoint)

    currentpos0 = odrv0.axis0.encoder.pos_estimate
    currentpos1 = odrv0.axis1.encoder.pos_estimate
    pos0 = msg.motor_commands[0].pos_setpoint
    pos1 = msg.motor_commands[1].pos_setpoint

    rospy.loginfo("Motor 0 is at: " + str(currentpos0) +
                  " should be " + str(pos0))

    odrv0.axis0.controller.pos_setpoint = pos0
    odrv0.axis1.controller.pos_setpoint = pos1


def listener():
    # Set up the motors!
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    use_mpc = rospy.get_param("/tail_controller/use_mpc")
    rospy.init_node('listener', anonymous=True)

    if(use_mpc):
        # Track mpc trajectory
        tail_topic = "/control/tail_command"
        rospy.Subscriber(tail_topic, LegCommand, mpc_callback, queue_size=1)
    else:
        # Feedback control
        state_topic = "/mcu/state/imu"
        rospy.Subscriber(state_topic, Imu, feedback_callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # put motors into idle when program is killed
    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    odrv0.axis1.requested_state = AXIS_STATE_IDLE


if __name__ == '__main__':
    listener()
