import roslaunch
import rospy

num = 100

for i in range(num):

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    launch_args = ['spirit_utils', 'spirit_gazebo.launch', 'paused:=false', 'world:=step_30cm', 'tail:=true', 'tail_type:=2']
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch.start()
    rospy.loginfo('Gazebo running')

    rospy.sleep(2.5)

    launch_args = ['spirit_utils', 'standing.launch']
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
    launch_2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_2.start()
    rospy.loginfo('Standing')

    rospy.sleep(10)

    launch_args = ['spirit_utils', 'planning.launch', 'global_planner:=twist', 'mpc_type:=nonlinear', 'logging:=true', 'tail:=true']
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
    launch_3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_3.start()
    rospy.loginfo('MPC running')

    rospy.sleep(50)

    launch.shutdown()
    launch_2.shutdown()
    launch_3.shutdown()

# for i in range(num):

#     uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#     roslaunch.configure_logging(uuid)

#     launch_args = ['spirit_utils', 'spirit_gazebo.launch', 'paused:=false', 'world:=step_20cm', 'tail:=true', 'tail_type:=3']
#     launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
#     launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
#     launch.start()
#     rospy.loginfo('Gazebo running')

#     rospy.sleep(2.5)

#     launch_args = ['spirit_utils', 'standing.launch']
#     launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
#     launch_2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
#     launch_2.start()
#     rospy.loginfo('Standing')

#     rospy.sleep(10)

#     launch_args = ['spirit_utils', 'planning.launch', 'global_planner:=twist', 'mpc_type:=nonlinear', 'logging:=true']
#     launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
#     launch_3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
#     launch_3.start()
#     rospy.loginfo('MPC running')

#     rospy.sleep(40)

#     launch.shutdown()
#     launch_2.shutdown()
#     launch_3.shutdown()

for i in range(num):

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    launch_args = ['spirit_utils', 'spirit_gazebo.launch', 'paused:=false', 'world:=step_30cm', 'tail_type:=0']
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch.start()
    rospy.loginfo('Gazebo running')

    rospy.sleep(2.5)

    launch_args = ['spirit_utils', 'standing.launch']
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
    launch_2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_2.start()
    rospy.loginfo('Standing')

    rospy.sleep(10)

    launch_args = ['spirit_utils', 'planning.launch', 'global_planner:=twist', 'mpc_type:=nonlinear', 'logging:=true']
    launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
    launch_3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    launch_3.start()
    rospy.loginfo('MPC running')

    rospy.sleep(50)

    launch.shutdown()
    launch_2.shutdown()
    launch_3.shutdown()
