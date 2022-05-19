import roslaunch
import rospy
import numpy as np

vel = 1.0
period = 0.36
num = 25
sample = 3
time_init = 2
time_stand = 2
time_walk = 20
world = ['world:=step_25cm', 'world:=step_30cm', 'world:=step_35cm', 'world:=step_40cm', 'world:=step_45cm', 'world:=step_50cm', 'world:=step_55cm', 'world:=step_60cm']
world = ['world:=step_20cm']

np.random.seed(0)

init_pos = np.linspace(-vel*period/2, vel*period/2, sample, endpoint=False)
# init_pos = np.array([init_pos]*int(num/sample)).T.flatten()
# init_pos = [init_pos[i] for i in [N-1]]
print(init_pos)

for w in world:

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

        launch_args = ['quad_utils', 'planning.launch', 'logging:=true', 'ac:=false', 'global_planner:=fgmp', 'leaping:=false']
        launch_pars = [(roslaunch.rlutil.resolve_launch_arguments(
            launch_args)[0], launch_args[2:])]
        launch_3 = roslaunch.parent.ROSLaunchParent(uuid, launch_pars)
        launch_3.start()
        rospy.loginfo('MPC running')

        rospy.sleep(time_walk)

        launch.shutdown()
        launch_2.shutdown()
        launch_3.shutdown()
