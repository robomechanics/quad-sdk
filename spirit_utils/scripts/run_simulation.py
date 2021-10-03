import roslaunch
import rospy

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch_args = ['spirit_utils', 'spirit_gazebo.launch', 'software:=false']
launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
launch.start()
launch_args = ['spirit_utils', 'visualization.launch']
launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
launch_2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
launch_2.start()
rospy.loginfo('Gazebo running')

rospy.sleep(5)

launch_args = ['spirit_utils', 'control.launch', 'controller:=open_loop']
launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
launch_3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
launch_3.start()
rospy.loginfo('Controller running')

launch_args = ['spirit_utils', 'standing.launch', 'mode:=standing']
launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
launch_4 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
launch_4.start()
rospy.loginfo('Standing')

rospy.sleep(5)

launch_4.shutdown()
launch_args = ['spirit_utils', 'standing.launch', 'mode:=walking']
launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(launch_args)[0], launch_args[2:])]
launch_4 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
launch_4.start()
rospy.loginfo('Walking')

rospy.sleep(50)

launch.shutdown()
launch_2.shutdown()
launch_3.shutdown()
