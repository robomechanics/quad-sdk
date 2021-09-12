#!/usr/bin/env python 
import pygame
import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

def run():
    rospy.init_node('twist_key_controller', anonymous=True)
    
    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    control_mode_pub = rospy.Publisher('/control/mode', UInt8, queue_size=1)

    lin_vel = 0.5
    ang_vel = 0.2

    pygame.init()
    screen = pygame.display.set_mode((300, 200))
    pygame.display.set_caption('Twist keyboard controller')

    # main loop
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

        keys_pressed = pygame.key.get_pressed()
      
        # Collect meta keys
        alt_pressed = keys_pressed[pygame.K_RALT] or keys_pressed[pygame.K_LALT]
        ctrl_pressed = keys_pressed[pygame.K_RCTRL] or keys_pressed[pygame.K_LCTRL]
        shift_pressed = keys_pressed[pygame.K_RSHIFT] or keys_pressed[pygame.K_LSHIFT]
        space_pressed = keys_pressed[pygame.K_SPACE]

        twist_vel = [0,0]
        twist_vel[0] -= keys_pressed[pygame.K_DOWN]
        twist_vel[0] += keys_pressed[pygame.K_UP]
        twist_vel[1] -= keys_pressed[pygame.K_LEFT]
        twist_vel[1] += keys_pressed[pygame.K_RIGHT]
        
        control_mode = -1
        if (keys_pressed[pygame.K_0]):
            control_mode = 0
        elif (keys_pressed[pygame.K_1]):
            control_mode = 1
        elif (keys_pressed[pygame.K_2]):
            control_mode = 2

        if (space_pressed):
            twist_pub.publish(Twist())
        elif (not twist_vel[0] == 0 or not twist_vel[1] == 0):
            twist_cmd = Twist()
            
            twist_cmd.linear.x = lin_vel*twist_vel[0]

            # Hold shift to strafe
            if shift_pressed:
                twist_cmd.linear.y = lin_vel*twist_vel[1]
            else:
                twist_cmd.angular.z = -ang_vel*twist_vel[1]

            twist_pub.publish(twist_cmd)

        if (not control_mode == -1):
            control_mode_pub.publish(control_mode)

        rate.sleep()


run()