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
od = odrive.find_any()
print("found an odrive")

while od.axis0.current_state != AXIS_STATE_IDLE or od.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

dump_errors(od, True)

start0 = od.axis0.encoder.pos_estimate
start1 = od.axis1.encoder.pos_estimate

# def flick(angular_velocity):
#     threshold = 5
#     amplitude = 20000
#     #rospy.loginfo("IMU angular velocity  x: [%f],  y: [%f],  z: [%f]" , data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)
#     velx = angular_velocity[0]
#     vely = angular_velocity[1]
    
#     #if speed is over the threshold filck the tail 
#     if abs(vely) > threshold:
#         od.axis0.controller.config.control_mode = 1
#         #A = get_amp(vely)od.axis0.controller.config.control_mode = 2
#         if vely > 0:
#             V = 5
#             od.axis0.controller.pos_setpoint = amplitude
#         else:
#             V = -5
#             od.axis0.controller.pos_setpoint = amplitude * -1

#     # return to near 0 position 
#     elif abs(od.axis0.encoder.pos_estimate)>65:
#         od.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL 
#         od.axis0.controller.pos_setpoint = 0
        
#     else:
#         od.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
#         od.axis0.controller.current_setpoint = 0 
#     #out of range / saftey stop
#     if(abs(od.axis0.encoder.pos_estimate) > 35000):
#         od.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
#         od.axis0.controller.current_setpoint = 0
#         print('out of range 0')

#     if abs(velx) > threshold:
#         od.axis1.controller.config.control_mode = 2
#         if vely > 0:
#             V = 5
#             od.axis1.controller.pos_setpoint = amplitude
#         else
#             V = -5
#             od.axis1.controller.pos_setpoint = amplitude * -1
#         '''
#         od.axis0.controller.vel_setpoint = V
#         while abs(od.axis0.encoder.pos_estimate) < amplitude:
#             od.axis0.controller.vel_setpoint = V
#         od.axis0.controller.vel = 0'''

#     # return to near 0 position 
#     elif abs(od.axis1.encoder.pos_estimate)>65+abs(start1):
#         print("is at " + str(od.axis1.encoder.pos_estimate))
#         od.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL 
#         od.axis1.controller.pos_setpoint = start1
#         rospy.loginfo("go to start")
#         print("start is" + str(start1))
#     else:
#         od.axis1.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
#         od.axis1.controller.current_setpoint = 0 
#     if abs(od.axis1.encoder.pos_estimate) > 35000:
#         od.axis1.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
#         od.axis1.controller.current_setpoint = 0
#         print('out of range 0')

def balance_controller(orientation, angular_velocity):
    # Should be replaced by encoder pos
    od.axis1.controller.input_pos = start1 + 50/(2*np.pi)*orientation[0]*0.25
    od.axis0.controller.input_pos = start0 + 50/(2*np.pi)*orientation[0]*0.25

    # Should be replaced by encoder vel
    # od.axis0.controller.vel_setpoint = angular_velocity[0]*0.5
    # od.axis1.controller.vel_setpoint = angular_velocity[1]*0.5

def feedback_callback(msg):
    # roll
    sinr_cosp = 2 * (msg.orientation.w * msg.orientation.x + msg.orientation.y * msg.orientation.z)
    cosr_cosp = 1 - 2 * (msg.orientation.x * msg.orientation.x + msg.orientation.y * msg.orientation.y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch
    sinp = 2 * (msg.orientation.w * msg.orientation.y - msg.orientation.z * msg.orientation.x)
    if np.abs(sinp) >= 1:
        pitch = np.sign(sinp)*np.pi/2
    else:
        pitch = np.arcsin(sinp)

    # yaw
    siny_cosp = 2 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y)
    cosy_cosp = 1 - 2 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    orientation = [roll, pitch, yaw]
    print("----------------orientation----------------")
    print(orientation)

    angular_velocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
    print("----------------angular_velocity----------------")
    print(angular_velocity)

    balance_controller(orientation, angular_velocity)

def mpc_callback(msg):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.motor_commands[1].pos_setpoint)
    
    currentpos0 = od.axis0.encoder.pos_estimate
    currentpos1 = od.axis1.encoder.pos_estimate
    pos0 =  msg.motor_commands[0].pos_setpoint
    pos1 =  msg.motor_commands[1].pos_setpoint

    rospy.loginfo("Motor 0 is at: " + str(currentpos0) + " should be " + str( pos0))

    od.axis0.controller.pos_setpoint = pos0
    od.axis1.controller.pos_setpoint = pos1
    
def listener():
    # Calibrate the motors (needed when running for the first time after being off or having an error)
    # calibrate = True

    # if calibrate:
    #     print("starting calibration...")
    #     od.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    #     od.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    #     while od.axis0.current_state != AXIS_STATE_IDLE or od.axis1.current_state != AXIS_STATE_IDLE:
    #          time.sleep(0.1)

    # Set up the motors! 
    od.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    od.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    # od.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    # od.axis0.trap_traj.config.vel_limit = 700000
    # od.axis0.trap_traj.config.accel_limit = 800000
    # od.axis0.trap_traj.config.decel_limit = 800000
    # od.axis0.controller.config.vel_limit  = 720000

    # od.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    # od.axis1.trap_traj.config.vel_limit = 700000
    # od.axis1.trap_traj.config.accel_limit = 800000
    # od.axis1.trap_traj.config.decel_limit = 800000
    # od.axis1.controller.config.vel_limit  = 720000

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

    #put motors into idle when program is killed 
    od.axis0.requested_state = AXIS_STATE_IDLE
    od.axis1.requested_state = AXIS_STATE_IDLE

if __name__ == '__main__':
    listener()
