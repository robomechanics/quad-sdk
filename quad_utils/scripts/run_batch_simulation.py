import roslaunch
import rospy
import numpy as np
import sys

# ==========
# Settings
# ==========

# Simulation initialization time
time_init = 3.5

# Standing time
time_stand = 7.5

# Actual walking and simulation time
time_walk = 45

# World parameter list
world_list = ['step_20cm', 'gap_40cm']

# Type name list
type_list = ['simple', 'complex', 'mixed', 'adaptive']

# Batch size for random initialization
batch_size = 10

# Random sample radius
random_radius = 0.1

# Prefix in the bag name
name_prefix = '05realtime'

# ==========
# Input
# ==========

# World index maps [0, 1] to [step_20cm, gap_40cm]
world_index = int(sys.argv[1])

# Type index maps [0, 3] to [Simple, Complex, Mixed, Adaptive], you still need to manully change the nmpc parameter now though
type_index = int(sys.argv[2])

# ==========
# Init
# ==========

# Guarentee deterministic
np.random.seed(0)

# Initial position list
init_radius = random_radius * np.random.rand(batch_size)
init_angle = 2 * np.pi * (np.random.rand(batch_size) - 0.5)
init_pos = np.array([1.0 + init_radius * np.cos(init_angle), init_radius * np.sin(init_angle)])

# Specify leap or not
leap_arg = 'leaping:=false'
if world_index == 0:
    leap_arg = 'leaping:=false'
elif world_index == 1:
    leap_arg = 'leaping:=true'

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
    launch_args = ['quad_utils', 'logging.launch', 'bag_name:='+world_list[world_index]+'_'+name_prefix+'_'+type_list[type_index]+'_'+str(i)]
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
        launch_args)[0], launch_args[2:])]
    launch_logging = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_logging.start()
    rospy.loginfo('Logging')

    # Wait walking
    rospy.sleep(time_walk)

    # Kill all the launch
    launch_gazebo.shutdown()
    launch_stand.shutdown()
    launch_planning.shutdown()
    launch_logging.shutdown()
