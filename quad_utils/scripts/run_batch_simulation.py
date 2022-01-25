import roslaunch
import rospy
import numpy as np

vel = 0.75
period = 0.36
num = 50
time_init = 5
time_stand = 7.5
time_walk = 32.5
world = ['world:=step_25cm']

np.random.seed(0)

init_pos = np.random.rand(num)*vel*period+6.0

for w in world:

    # for i in range(num):

    #     uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #     roslaunch.configure_logging(uuid)

    #     launch_args = ['quad_utils', 'quad_gazebo.launch', 'paused:=false',
    #                 w, 'tail:=true', 'tail_type:=2', 'x_init:='+str(init_pos[i])]
    #     launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
    #         launch_args)[0], launch_args[2:])]
    #     launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    #     launch.start()
    #     rospy.loginfo('Gazebo running')

    #     rospy.sleep(time_init)

    #     launch_args = ['quad_utils', 'standing.launch']
    #     launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
    #         launch_args)[0], launch_args[2:])]
    #     launch_2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    #     launch_2.start()
    #     rospy.loginfo('Standing')

    #     rospy.sleep(time_stand)

    #     launch_args = ['quad_utils', 'planning.launch',
    #                 'logging:=true', 'tail:=true']
    #     launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
    #         launch_args)[0], launch_args[2:])]
    #     launch_3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    #     launch_3.start()
    #     rospy.loginfo('MPC running')

    #     rospy.sleep(time_walk)

    #     launch.shutdown()
    #     launch_2.shutdown()
    #     launch_3.shutdown()

    # for i in range(num):

    #     uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #     roslaunch.configure_logging(uuid)

    #     launch_args = ['quad_utils', 'quad_gazebo.launch', 'paused:=false',
    #                    w, 'tail:=true', 'tail_type:=3', 'x_init:='+str(init_pos[i])]
    #     launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
    #         launch_args)[0], launch_args[2:])]
    #     launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    #     launch.start()
    #     rospy.loginfo('Gazebo running')

    #     rospy.sleep(time_init)

    #     launch_args = ['quad_utils', 'standing.launch']
    #     launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
    #         launch_args)[0], launch_args[2:])]
    #     launch_2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    #     launch_2.start()
    #     rospy.loginfo('Standing')

    #     rospy.sleep(time_stand)

    #     launch_args = ['quad_utils', 'planning.launch',
    #                    'logging:=true']
    #     launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
    #         launch_args)[0], launch_args[2:])]
    #     launch_3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
    #     launch_3.start()
    #     rospy.loginfo('MPC running')

    #     rospy.sleep(time_walk)

    #     launch.shutdown()
    #     launch_2.shutdown()
    #     launch_3.shutdown()

    for i in range(num):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_args = ['quad_utils', 'quad_gazebo.launch', 'paused:=false',
                    w, 'x_init:='+str(init_pos[i])]
        launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
            launch_args)[0], launch_args[2:])]
        launch = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
        launch.start()
        rospy.loginfo('Gazebo running')

        rospy.sleep(time_init)

        launch_args = ['quad_utils', 'standing.launch']
        launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
            launch_args)[0], launch_args[2:])]
        launch_2 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
        launch_2.start()
        rospy.loginfo('Standing')

        rospy.sleep(time_stand)

        launch_args = ['quad_utils', 'planning.launch', 'logging:=true']
        launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
            launch_args)[0], launch_args[2:])]
        launch_3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
        launch_3.start()
        rospy.loginfo('MPC running')

        rospy.sleep(time_walk)

        launch.shutdown()
        launch_2.shutdown()
        launch_3.shutdown()
