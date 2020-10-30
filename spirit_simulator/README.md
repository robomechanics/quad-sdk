# kodlab_gazebo
ROS package for the simulation of robotic platforms using Gazebo

Maintainer: Vasileios Vasilopoulos (<vvasilo@seas.upenn.edu>)

## General notes
1. This ROS package can be used in conjunction with the Ghost Robotics SDK artifacts, which are independently distributed by Ghost Robotics.
2. At this point, there is no open source Vision60 control example in the SDK, but once released the following procedure should be identical. For now, you can just launch the Vision60 simulation as explained below.
3. The simulation depends a lot on friction parameters. These parameters can be configured in the URDF xacro files ([minitaur_gazebo.urdf.xacro](minitaur_description/urdf/minitaur_gazebo.urdf.xacro), [vision60_gazebo.urdf.xacro](vision60_description/urdf/vision60_gazebo.urdf.xacro) and [spirit_gazebo.urdf.xacro](spirit_description/urdf/spirit_gazebo.urdf.xacro)).

## Running the simulation
1. For the package to work properly, make sure you have installed the following packages:
```
$ sudo apt-get install ros-melodic-controller-manager ros-melodic-joint-state-controller ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-effort-controllers python-catkin-tools
```

2. For some illustrations, we also use the package [3DGEMS](http://data.nvision2.eecs.yorku.ca/3DGEMS/). Download all the subfolders mentioned on the package website and put them in a separate folder.

3. Copy the package to your ROS workspace folder
```
$ cp -r <artifacts_location>/thirdparty/kodlab_gazebo <catkin_ws_location>/src
```

4. The following line needs to be added in `~/.bashrc` to allow for proper Gazebo model detection
```
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:<catkin_ws_location>/src/kodlab_gazebo/minitaur_description/sdf:<catkin_ws_location>/src/kodlab_gazebo/minitaur_description:<catkin_ws_location>/src/kodlab_gazebo/vision60_description:<catkin_ws_location>/src/kodlab_gazebo/spirit_description
```

5. Also, the following line needs to be added in `~/.bashrc` to allow for proper use of [3DGEMS](http://data.nvision2.eecs.yorku.ca/3DGEMS/)
```
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:<3dgems_models_location>/decoration:<3dgems_models_location>/earthquake:<3dgems_models_location>/electronics:<3dgems_models_location>/food:<3dgems_models_location>/furniture:<3dgems_models_location>/kitchen:<3dgems_models_location>/miscellaneous:<3dgems_models_location>/shapes:<3dgems_models_location>/stationery:<3dgems_models_location>/tools
```

6. Configure [ghost.world](gazebo_scripts/worlds/ghost.world) (or any other [world](gazebo_scripts/worlds) file) to include anything you want to show up in your simulation. In this file, you can also configure `max_step_size` and `real_time_update_rate` for the simulation. Usually a `max_step_size` of 0.0005 works for Minitaur. Vision needs a smaller value for contact forces (0.00005 should be enough). Currently, `real_time_update_rate` is configured to 2000Hz.

7. Build all the packages
```
$ cd <catkin_ws_location>
$ catkin build
```

8. To launch the simulation, build the package and run:
```
$ roslaunch gazebo_scripts minitaur_gazebo.launch
```
for standalone Minitaur, or
```
$ roslaunch gazebo_scripts minitaur_sensor_gazebo.launch
```
for Minitaur equipped with a sensor head, or 
```
$ roslaunch gazebo_scripts vision60_gazebo.launch
```
for Vision60, or
```
$ roslaunch gazebo_scripts spirit_gazebo.launch
```
for Spirit.

9. (Optional) If you want to control the robot using the Ghost SDK, compile and run the corresponding Ghost Robotics SDK script. For this, please use the [Makefile example](extras/Makefile) provided here (the format might change in the future). The simulation exposes 3 ROS topics for control: `/behaviorId`, `/behaviorMode` and `/twist`, and several others for checking the robot state: `/<robot_name>/state/imu`, `/<robot_name>/state/batteryState`, `/<robot_name>/state/behaviorId`, `/<robot_name>/state/behaviorMode`, `/<robot_name>/state/joint`, `/<robot_name>/state/pose`. As an example following the Ghost Robotics SDK FirstHop example for Minitaur, the overall process should look like that:
```
$ mv <artifacts_location>/examples/FirstHop/Makefile <artifacts_location>/examples/FirstHop/Makefile.bk
$ cp <artifacts_location>/thirdparty/kodlab_gazebo/extras/Makefile <artifacts_location>/examples/FirstHop/Makefile
$ cd <artifacts_location>/examples/FirstHop
$ make
$ ./FirstHop_MINITAUR_E
```

## Converting Minitaur's URDF to SDF
The most important requirement to setup the simulation is to properly convert the Minitaur's URDF file to an SDF file. We have a URDF xacro file ([minitaur_gazebo.urdf.xacro](minitaur_description/urdf/minitaur_gazebo.urdf.xacro)) in the [urdf folder](minitaur_description/urdf) and properly converted URDF and SDF files (see [minitaur_constrained.sdf](minitaur_description/sdf/minitaur_constrained/minitaur_constrained.sdf)). If you need to modify any of the above, please run the following commands to convert the xacro file to URDF
```bash
$ python xacro.py minitaur_gazebo.urdf.xacro > minitaur_gazebo.urdf
```
and subsequently to SDF using
```bash
$ gz sdf -p minitaur_gazebo.urdf > minitaur_gazebo.sdf
```

However, you also need to do the following after the conversion:
1. Make sure to add the lines below to the end of the SDF file (before the model termination). These lines realize the 5-bar mechanism constraint for each leg (hence the name `constrained`).
```
    <joint name='constraint_back_left' type='revolute'>
      <parent>lower_leg_back_leftL_link</parent>
      <child>lower_leg_back_leftR_link</child>
      <pose frame=''>0 0 0.19 0 -0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <use_parent_model_frame>0</use_parent_model_frame>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
          <effort>-1</effort>
          <velocity>-1</velocity>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
          <damping>0</damping>
        </dynamics>
      </axis>
      <physics>
        <ode>
          <limit>
            <cfm>0</cfm>
            <erp>0.2</erp>
          </limit>
          <suspension>
            <cfm>0</cfm>
            <erp>0.2</erp>
          </suspension>
        </ode>
      </physics>
    </joint>
    <joint name='constraint_back_right' type='revolute'>
      <parent>lower_leg_back_rightR_link</parent>
      <child>lower_leg_back_rightL_link</child>
      <pose frame=''>0 0 0.19 0 -0 0</pose>
      <axis>
        <xyz>-0 -1 -0</xyz>
        <use_parent_model_frame>0</use_parent_model_frame>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
          <effort>-1</effort>
          <velocity>-1</velocity>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
          <damping>0</damping>
        </dynamics>
      </axis>
      <physics>
        <ode>
          <limit>
            <cfm>0</cfm>
            <erp>0.2</erp>
          </limit>
          <suspension>
            <cfm>0</cfm>
            <erp>0.2</erp>
          </suspension>
        </ode>
      </physics>
    </joint>
    <joint name='constraint_front_left' type='revolute'>
      <parent>lower_leg_front_leftL_link</parent>
      <child>lower_leg_front_leftR_link</child>
      <pose frame=''>0 0 0.19 0 -0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <use_parent_model_frame>0</use_parent_model_frame>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
          <effort>-1</effort>
          <velocity>-1</velocity>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
          <damping>0</damping>
        </dynamics>
      </axis>
      <physics>
        <ode>
          <limit>
            <cfm>0</cfm>
            <erp>0.2</erp>
          </limit>
          <suspension>
            <cfm>0</cfm>
            <erp>0.2</erp>
          </suspension>
        </ode>
      </physics>
    </joint>
    <joint name='constraint_front_right' type='revolute'>
      <parent>lower_leg_front_rightR_link</parent>
      <child>lower_leg_front_rightL_link</child>
      <pose frame=''>0 0 0.19 0 -0 0</pose>
      <axis>
        <xyz>-0 -1 -0</xyz>
        <use_parent_model_frame>0</use_parent_model_frame>
        <limit>
          <lower>-1.79769e+308</lower>
          <upper>1.79769e+308</upper>
          <effort>-1</effort>
          <velocity>-1</velocity>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
          <damping>0</damping>
        </dynamics>
      </axis>
      <physics>
        <ode>
          <limit>
            <cfm>0</cfm>
            <erp>0.2</erp>
          </limit>
          <suspension>
            <cfm>0</cfm>
            <erp>0.2</erp>
          </suspension>
        </ode>
      </physics>
    </joint>
```
2. For each leg, rename all the toe collision descriptions to match the corresponding toe (e.g., for the front left leg, rename `lower_leg_front_leftL_link_fixed_joint_lump__toe0_collision_1` to `toe0_collision`), and also add the following lines immediately below this collision tag (change `0` to match each toe):
```
<sensor name="toe0_contact" type="contact">
    <plugin name="toe0_plugin" filename="libcontact.so"/>
    <contact>
        <collision>toe0_collision</collision>
    </contact>
</sensor>
```
3. To help Minitaur with turning, you need to make joints '8', '9', '10', '11', '12', '13', '14' and '15' universal, to match the following format example. Make sure that the sign of 1 in `<xyz>` for both axes is positive for joints '9', '11', '13' and '15' (and with limits -2.091 and 1.049) and negative for '8', '10', '12' and '14' (and with limits -1.049 and 2.091).
```
    <joint name='9' type='universal'>
      <child>lower_leg_front_leftR_link</child>
      <parent>motor_front_leftR_link</parent>
      <axis>
        <xyz>-0 1 -0</xyz>
        <limit>
          <lower>-2.091</lower>
          <upper>1.049</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
        <use_parent_model_frame>1</use_parent_model_frame>
      </axis>
      <axis2>
        <xyz>1 0 0</xyz>
        <limit>
          <lower>-0.02</lower>
          <upper>0.02</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
        <use_parent_model_frame>1</use_parent_model_frame>
      </axis2>
    </joint>
```
4. Make sure to move the SDF file to the [minitaur_constrained folder](minitaur_description/sdf/minitaur_constrained) and rename it from `minitaur_gazebo.sdf` to `minitaur_constrained.sdf`.
5. Rename the model name in [minitaur_constrained.sdf](minitaur_description/sdf/minitaur_constrained/minitaur_constrained.sdf) to `minitaur_constrained`. Do the same thing for the namespace in `libgazebo_ros_control.so`.

## Minitaur with sensor head
We have included an SDF file describing Minitaur equipped with a sensor head on top (see [minitaur_sensor.sdf](minitaur_description/sdf/minitaur_sensor/minitaur_sensor.sdf)). If you modify any of the xacro files describing Minitaur or want to generate this SDF file on your own, follow the steps above to generate [minitaur_constrained.sdf](minitaur_description/sdf/minitaur_constrained/minitaur_constrained.sdf) first. Then:
1. Copy the SDF file to the [minitaur_sensor folder](minitaur_description/sdf/minitaur_sensor) and rename it from `minitaur_constrained.sdf` to `minitaur_sensor.sdf`.
2. To add the sensor head, include the following lines at the end of the SDF file (before the model termination):
```
    <include>
      <uri>model://sensor_head</uri>
      <pose>0 0 0.04 0 0 0</pose>
    </include>
    <joint name="sensor_head_joint" type="fixed">
      <child>sensor_head::base</child>
      <parent>base_chassis_link_dummy</parent>
    </joint>
```
The sensor head description is included in [sensor_head.sdf](minitaur_description/sdf/sensor_head/sensor_head.sdf) and you are free to modify it as needed. 

**NOTE**: For the LIDAR you can set the sensor type to either `gpu_ray` or `ray` depending on whether GPU acceleration is available or not. The corresponding plugin filenames to use are `libgazebo_ros_gpu_laser.so` and `libgazebo_ros_laser.so`. 

## Converting Vision60's URDF to SDF
This process is more straightforward and we have already included properly converted URDF and SDF files. We have a URDF xacro file ([vision60_gazebo.urdf.xacro](vision60_description/urdf/vision60_gazebo.urdf.xacro)) in the urdf folder that can be converted to URDF with
```bash
$ python xacro.py vision60_gazebo.urdf.xacro > vision60_gazebo.urdf
```
and subsequently to SDF using
```bash
$ gz sdf -p vision60_gazebo.urdf > ../sdf/vision60.sdf
```

After the conversion:
1. For each leg, rename all the toe collision descriptions to match the corresponding toe (use `toe0_collision`, `toe1_collision` as in the Minitaur SDF), and also add the following lines immediately below this collision tag (change `0` to match each toe):
```
<sensor name="toe0_contact" type="contact">
    <plugin name="toe0_plugin" filename="libcontact.so"/>
    <contact>
        <collision>toe0_collision</collision>
    </contact>
</sensor>
```
2. Make sure to move the SDF file to the [vision60 SDF folder](vision60_description/sdf) and rename it to vision60.sdf.

## Converting Spirit's URDF to SDF
We have a URDF xacro file ([spirit_gazebo.urdf.xacro](spirit_description/urdf/spirit_gazebo.urdf.xacro)) in the urdf folder that can be converted to URDF with
```bash
$ python xacro.py spirit_gazebo.urdf.xacro > spirit_gazebo.urdf
```
and subsequently to SDF using
```bash
$ gz sdf -p spirit_gazebo.urdf > ../sdf/spirit.sdf
```

After the conversion:
1. For each leg, rename all the toe collision descriptions to match the corresponding toe (use `toe0_collision`, `toe1_collision` as in the Minitaur SDF), and also add the following lines immediately below this collision tag (change `0` to match each toe):
```
<sensor name="toe0_contact" type="contact">
    <plugin name="toe0_plugin" filename="libcontact.so"/>
    <contact>
        <collision>toe0_collision</collision>
    </contact>
</sensor>
```
2. Make sure to move the SDF file to the [spirit SDF folder](spirit_description/sdf) and rename it to spirit.sdf.

## Other

Except for the robots, we also include other useful descriptions of objects for autonomous tasks.

### AprilTags

We have included the Gazebo model descriptions of some members of the 36h11 [AprilTag](https://april.eecs.umich.edu/software/apriltag) family. Make sure to add the descriptions to your `GAZEBO_MODEL_PATH` by adding the following line to your `~/.bashrc` file:
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<catkin_ws_location>/src/kodlab_gazebo/other/apriltags_description
```

If you want to generate your own descriptions, you can download pre-generated tags [here](https://april.eecs.umich.edu/software/apriltag). Those tags (`.png` images) are quite small, so you might need to enlarge them (we suggest [gimp](https://www.gimp.org/) - make sure to scale the images with `None` as the interpolation method). The package we used for the automated SDF model generation can be found [here](https://github.com/vvasilo/gazebo_models). The size of the generated tags (including the white margin) was set to 85mm. The dimension ratio between the tag's black square and the outside box (that includes the white margin) is approximately 0.8; this can be used to appropriately scale your tags.
