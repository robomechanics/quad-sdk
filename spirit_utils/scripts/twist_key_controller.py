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

    pygame.init()
    screen = pygame.display.set_mode((300, 200))
    pygame.display.set_caption('Twist keyboard controller')

    # main loop
    rate = rospy.Rate(25) # 10hz
    while not rospy.is_shutdown():
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

        keys_pressed = pygame.key.get_pressed()
        
        h = 1
        twist_vel = [0,0]
        twist_vel[0] -= h*keys_pressed[pygame.K_DOWN]
        twist_vel[0] += h*keys_pressed[pygame.K_UP]
        twist_vel[1] -= h*keys_pressed[pygame.K_LEFT]
        twist_vel[1] += h*keys_pressed[pygame.K_RIGHT]
        
        control_mode = -1
        if (keys_pressed[pygame.K_0]):
            control_mode = 0
        elif (keys_pressed[pygame.K_1]):
            control_mode = 1
        elif (keys_pressed[pygame.K_2]):
            control_mode = 2

        if (not twist_vel[0] == 0 or not twist_vel[1] == 0):
          twist_cmd = Twist()
          twist_cmd.linear.x = twist_vel[0]
          twist_cmd.linear.y = 0
          twist_cmd.angular.z = twist_vel[1]
          twist_pub.publish(twist_cmd)

        if (not control_mode == -1):
            control_mode_pub.publish(control_mode)

        rate.sleep()


run()