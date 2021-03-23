#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from spirit_msgs.msg import LegCommand
import odrive
from odrive.enums import *


print("finding an odrive...")
od = odrive.find_any()
print("found an odrive")


def callback(msg):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.motor_commands[1].pos_setpoint)

    pos0 =  msg.motor_commands[0].pos_setpoint
    pos1 =  msg.motor_commands[1].pos_setpoint
    od.axis0.controller.pos_setpoint = pos0
    od.axis1.controller.pos_setpoint = pos1
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    calibrate = True
    #Calibrate the motors (needed when running for the first time after being off or having an error)


    if(calibrate):
        od.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        od.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while od.axis0.current_state != AXIS_STATE_IDLE or od.axis1.current_state != AXIS_STATE_IDLE:
             time.sleep(0.1)

    #set up the motors! 
    od.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    od.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    od.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    od.axis0.trap_traj.config.vel_limit = 700000
    od.axis0.trap_traj.config.accel_limit = 800000
    od.axis0.trap_traj.config.decel_limit = 800000
    od.axis0.controller.config.vel_limit  = 720000
    od.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    od.axis1.trap_traj.config.vel_limit = 700000
    od.axis1.trap_traj.config.accel_limit = 800000
    od.axis1.trap_traj.config.decel_limit = 800000
    od.axis1.controller.config.vel_limit  = 720000
    start0 = od.axis0.encoder.pos_estimate
    start1 = od.axis1.encoder.pos_estimate

    rospy.init_node('listener', anonymous=True)
    tail_topic = "/control/tail_command"
    rospy.Subscriber(tail_topic, LegCommand, callback, queue_size=1)



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    #put motors into idle when program is killed 
    od.axis0.requested_state = AXIS_STATE_IDLE
    od.axis1.requested_state = AXIS_STATE_IDLE



if __name__ == '__main__':
    listener()
