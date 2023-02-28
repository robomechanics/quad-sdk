#!/usr/bin/env python 

import rospy
import geometry_msgs.msg
import nav_msgs
import tf2_ros

def main_program():
    """ Main function initializes node and subscribers and starts
        the ROS loop. """
    rospy.init_node('camera_pose_frame_tf_listener')
    pub_topic = rospy.get_param('topics/mocap')
    #sub_topic = rospy.get_param('topics/camera_odom_sample')
    from_frame = 'map'
    to_frame = 'robot_1_ground_truth/body'

    tfBuffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tfBuffer)
    publisher = rospy.Publisher(
        pub_topic, geometry_msgs.msg.PoseStamped, queue_size=10)

    # Set callback and start spinning
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(from_frame, to_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("fail to lookup transfrom")
            r.sleep()
            continue

    # Create and fill pose message for publishing
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = from_frame
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = trans.transform.translation.z
        pose.pose.orientation.x = trans.transform.rotation.x
        pose.pose.orientation.y = trans.transform.rotation.y
        pose.pose.orientation.z = trans.transform.rotation.z
        pose.pose.orientation.w = trans.transform.rotation.w
        publisher.publish(pose)

        r.sleep()

if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException:
        pass
