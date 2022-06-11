import roslaunch
import rospy
import numpy as np
import sys
from geometry_msgs.msg import Wrench, Vector3, Point
from gazebo_msgs.srv import ApplyBodyWrench

# ==========
# Settings
# ==========

# Simulation initialization time
time_init = 3.5

# Standing time
time_stand = 12
time_stand = 12*2/5

# Actual walking and simulation time
time_walk = 45
time_walk = 45*2/5

# World parameter list
world_list = ['step_20cm', 'gap_40cm', 'flat']

# Type name list
type_list = ['simple', 'complex', 'mixed', 'adaptive']

# Batch size for random initialization
batch_size = 10

# Prefix in the bag name

# Disturbance settings
apply_wrench = [0., 6500., 0., 0., 0., 0.] # fx, fy, fz, tx, ty, tz in body frame
apply_time = 4  # sec in gazebo
apply_duration = (0.001)*1e9  # sec in gazebo

# ==========
# Input
# ==========

# World index maps [0, 1] to [step_20cm, gap_40cm]
world_index = int(sys.argv[1])

# Type index maps [0, 3] to [Simple, Complex, Mixed, Adaptive], you still need to manully change the nmpc parameter now though
type_index = int(sys.argv[2])

# Specify leap or not
leap_arg = 'leaping:=false'
if world_index == 0:
    leap_arg = 'leaping:=false'
    time_stand = 12*2/5
    time_walk = 45*2/5
    name_prefix = '05realtime'
    random_radius = 0.2
    start_state_x = 1.0
elif world_index == 1:
    leap_arg = 'leaping:=true'
    time_stand = 8.5
    time_walk = 35
    name_prefix = '02realtime'
    random_radius = 0.2
    start_state_x = 0.9
elif world_index == 2:
    leap_arg = 'leaping:=false'
    time_stand = 5.0
    time_walk = 10.0
    name_prefix = '10realtime'
    random_radius = 0
    start_state_x = 0

# ==========
# Init
# ==========

# Guarantee deterministic
np.random.seed(0)

# Initial position list
init_radius = 2*random_radius * (np.random.rand(batch_size) - 0.5)
init_angle = 0  # 2 * np.pi * (np.random.rand(batch_size) - 0.5)
init_pos = np.array([start_state_x + init_radius *
                    np.cos(init_angle), init_radius * np.sin(init_angle)])

# ==========
# Functions
# ==========


def apply_body_wrench_client(body_name, reference_frame, reference_point, wrench,
                             start_time, duration):
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        apply_body_wrench = rospy.ServiceProxy(
            '/gazebo/apply_body_wrench', ApplyBodyWrench)
        apply_body_wrench(body_name, reference_frame, reference_point, wrench,
                          start_time, duration)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

# ==========
# Sim
# ==========


for i in range(batch_size):

    # ROS init
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Launch gazebo
    launch_args = ['quad_utils', 'quad_gazebo.launch', 'paused:=false', 'rviz_gui:=false',
                   'world:='+world_list[world_index], 'x_init:='+str(init_pos[0, i]), 'y_init:='+str(init_pos[1, i])]
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch_gazebo = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_gazebo.start()
    rospy.loginfo('Gazebo running')

    # Wait gazebo init
    rospy.sleep(time_init)

    # Stand robot
    launch_args = ['quad_utils', 'standing.launch']
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch_stand = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_stand.start()
    rospy.loginfo('Standing')

    # Wait standing
    rospy.sleep(time_stand)

    # Start planning
    launch_args = ['quad_utils', 'planning.launch',
                   'logging:=false', leap_arg]
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch_planning = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_planning.start()
    rospy.loginfo('Planning')

    # Start logging
    launch_args = ['quad_utils', 'logging.launch', 'bag_name:=' +
                   world_list[world_index]+'_'+name_prefix+'_'+type_list[type_index]+'_'+str(i)]
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch_logging = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_logging.start()
    rospy.loginfo('Logging')

    # Apply disturbance settings
    body_name = 'spirit::body'
    reference_frame = 'spirit::body'
    reference_point = Point(x=0, y=0, z=0)
    wrench = Wrench(force=Vector3(x=apply_wrench[0], y=apply_wrench[1], z=apply_wrench[2]), torque=Vector3(
        x=apply_wrench[3], y=apply_wrench[4], z=apply_wrench[5]))
    start_time = rospy.Time(secs=(time_stand + apply_time), nsecs=0)
    duration = rospy.Duration(secs=0, nsecs=apply_duration)
    apply_body_wrench_client(body_name, reference_frame,
                             reference_point, wrench, start_time, duration)

    # Wait walking
    rospy.sleep(time_walk)

    # Kill all the launch
    launch_gazebo.shutdown()
    launch_stand.shutdown()
    launch_planning.shutdown()
    launch_logging.shutdown()
