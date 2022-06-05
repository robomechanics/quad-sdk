import rospy
import geometry_msgs
import nav_msgs
import tf

def callback(msg):
    """Listens to a transform between from_frame and to_frame and publishes it
       as a pose with a zero covariance."""
    global publisher, tf_listener, from_frame, to_frame

    # Listen to transform and throw exception if the transform is not
    # available.
    try:
        (trans, rot) = tf_listener.lookupTransform(
            from_frame, to_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException):
        return

    # Create and fill pose message for publishing
    pose = geometry_msgs.PoseStamped()
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = from_frame
    pose.pose.position.x = trans[0]
    pose.pose.position.y = trans[1]
    pose.pose.position.z = trans[2]
    pose.pose.orientation.x = rot[0]
    pose.pose.orientation.y = rot[1]
    pose.pose.orientation.z = rot[2]
    pose.pose.orientation.w = rot[3]

    publisher.publish(pose)


def main_program():
    """ Main function initializes node and subscribers and starts
        the ROS loop. """
    global publisher, tf_listener, from_frame, to_frame
    rospy.init_node('camera_pose_frame_tf_listener')
    pub_topic = rospy.get_param('topics/mocap')
    #sub_topic = rospy.get_param('topics/camera_odom_sample')
    from_frame = 'map'
    to_frame = 'ground_truth/body'

    tf_listener = tf.TransformListener()
    publisher = rospy.Publisher(
        pub_topic, geometry_msgs.PoseStamped, queue_size=10)
    # rospy.Subscriber(sub_topic, nav_msgs.Odometry, callback)

    # Set callback and start spinning
    rospy.Timer(rospy.Duration(0.05), callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException:
        pass