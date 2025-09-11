2.4.0 (2024-03-08)
------------------
* Fixes for flake8. (`#29 <https://github.com/ros2/teleop_twist_keyboard/issues/29>`_)
* Show command to change topic name on readme (`#27 <https://github.com/ros2/teleop_twist_keyboard/issues/27>`_)
* Remove url for ros1 on package.xml (`#28 <https://github.com/ros2/teleop_twist_keyboard/issues/28>`_)
* Added TwistStamped option (`#26 <https://github.com/ros2/teleop_twist_keyboard/issues/26>`_)
* Switch to underscores for setup.cfg. (`#25 <https://github.com/ros2/teleop_twist_keyboard/issues/25>`_)
* Contributors: Asuki Kono, Chris Lalancette, agyoungs

2.3.2 (2020-05-12)
------------------
* Add Windows support to teleop_twist_keyboard. (`#24 <https://github.com/ros2/teleop_twist_keyboard/issues/24>`_)
* Contributors: Chris Lalancette

2.3.1 (2019-10-03)
------------------
* Stop using deprecated APIs. (`#22 <https://github.com/ros2/teleop_twist_keyboard/issues/22>`_)
* Contributors: Chris Lalancette

2.3.0 (2019-04-19)
------------------
* changing QoS to default (`#18 <https://github.com/ros2/teleop_twist_keyboard/issues/18>`_)
* Contributors: thorstink

2.1.1 (2018-06-26)
------------------

2.1.0 (2018-06-26)
------------------
* set zip_safe to avoid warning during installation (`#14 <https://github.com/ros2/teleop_twist_keyboard/issues/14>`_)
* Update the maintainer to me. (`#13 <https://github.com/ros2/teleop_twist_keyboard/issues/13>`_)
* fix data files install path (`#12 <https://github.com/ros2/teleop_twist_keyboard/issues/12>`_)
* remove test_suite, add pytest as test_requires (`#11 <https://github.com/ros2/teleop_twist_keyboard/issues/11>`_)
* use ros2 run to launch whichever the installation used (`#8 <https://github.com/ros2/teleop_twist_keyboard/issues/8>`_)
* Fix teleop_twist_keyboard to have a setup.cfg. (`#9 <https://github.com/ros2/teleop_twist_keyboard/issues/9>`_)
* Make sure to add teleop_twist_keyboard to ament index. (`#6 <https://github.com/ros2/teleop_twist_keyboard/issues/6>`_)
* use OSI website as reference for license (`#2 <https://github.com/ros2/teleop_twist_keyboard/issues/2>`_)
* ROS 2 port (`#1 <https://github.com/ros2/teleop_twist_keyboard/issues/1>`_)
* Contributors: Adam Allevato, Austin, Chris Lalancette, Dirk Thomas, Mikael Arguedas

0.6.1 (2018-05-02)
------------------
* Merge pull request `#11 <https://github.com/ros-teleop/teleop_twist_keyboard/issues/11>`_ from MatthijsBurgh/patch-1
  Correct exception handling; Python3 print compatible
* import print from future
* Print python3 compatible
* correct Exception handling
* Merge pull request `#7 <https://github.com/ros-teleop/teleop_twist_keyboard/issues/7>`_ from lucasw/speed_params
  set linear and turn speed via rosparams
* Using tabs instead of spaces to match rest of file
* set linear and turn speed via rosparams
* Contributors: Austin, Lucas Walter, Matthijs van der Burgh

0.6.0 (2016-03-21)
------------------
* Better instruction formatting
* support holonomic base, send commmand with keyboard down
* Fixing queue_size warning
* Create README.md
* Update the description string in package.xml
* Contributors: Austin, Kei Okada, LiohAu, Mike Purvis, kk6axq, trainman419

0.5.0 (2014-02-11)
------------------
* Initial import from brown_remotelab
* Convert to catkin
