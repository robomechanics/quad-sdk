#! /usr/bin/env python


from rospy import Time
import time
import rospy
from rosgraph_msgs.msg import Clock

node_name = "pybullet_clock"

step_rate = 1000
clock_pub_name = "/clock"
# clock_pub_name = "/double_clock"
time_ratio = 1.0


if __name__ == '__main__':
    rospy.init_node(node_name)
    clock_pub = rospy.Publisher(clock_pub_name,Clock,queue_size=5)
    starttime = time.time_ns()
    while not rospy.is_shutdown():
        tnew = (time.time_ns()-starttime)*time_ratio
        tsec = int(tnew/1e9)
        tnsec = int(int((tnew-tsec*1e9)/1e6)*1e6) # By default publish by interval of 1ms
        new_rostime = rospy.Time(tsec,tnsec)
        clock_pub.publish(new_rostime)
        time.sleep(0.001/time_ratio)
    rospy.spin()
