^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package teleop_twist_joy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.1 (2024-06-17)
------------------
* Added Github action (`#48 <https://github.com/ros2/teleop_twist_joy/issues/48>`_)
* Add an option to publish TwistStamped (`#42 <https://github.com/ros2/teleop_twist_joy/issues/42>`_)
* Add support for PDP joysticks (`#41 <https://github.com/ros2/teleop_twist_joy/issues/41>`_)
  * Add support for PDP joysticks
* Contributors: Alejandro Hernández Cordero, Bonolo Mathibela, Tamaki Nishino

2.6.0 (2023-06-07)
------------------
* add inverted reverse param (`#35 <https://github.com/ros2/teleop_twist_joy/issues/35>`_)
* [rolling] Update maintainers - 2022-11-07 (`#33 <https://github.com/ros2/teleop_twist_joy/issues/33>`_)
* Enable uncrustify and cpplint.
* Cleanup CMakeLists.txt.
* Remove checking of types from parameter_callback.
* Install includes to include/${PROJECT_NAME} (`#30 <https://github.com/ros2/teleop_twist_joy/issues/30>`_)
* joy_vel argument (`#29 <https://github.com/ros2/teleop_twist_joy/issues/29>`_)
* Contributors: Audrow Nash, Chris Lalancette, Máté, Raffaello Bonghi, Shane Loretz

2.4.3 (2021-08-02)
------------------
* Fix the launch file to use 'executable'. (`#28 <https://github.com/ros2/teleop_twist_joy/issues/28>`_)
* fix launch notation (`#26 <https://github.com/ros2/teleop_twist_joy/issues/26>`_)
* Contributors: Chris Lalancette, Shigeki Kobayashi

2.4.2 (2021-03-18)
------------------
* Update README to reflect changes to config parameters. (fixes `#23 <https://github.com/ros2/teleop_twist_joy/issues/23>`_) (`#24 <https://github.com/ros2/teleop_twist_joy/issues/24>`_)
* Contributors: Josh Newans

2.4.1 (2020-12-01)
------------------
* Add parameter to enable/disable requiring the enable button to be held for motion (`#21 <https://github.com/ros2/teleop_twist_joy/issues/21>`__)
* Contributors: Chris Lalancette, kgibsonjca

2.4.0 (2020-11-09)
------------------
* Switch to modern ReadyToTest for the tests.
* Switch from node_executable -> executable for Foxy.
* Update README for Ros2 (`#17 <https://github.com/ros2/teleop_twist_joy/issues/17>`_) (`#18 <https://github.com/ros2/teleop_twist_joy/issues/18>`_)
* Contributors: Chris Lalancette, nfry321

2.3.0 (2020-08-05)
------------------
* Make Parameters dynamic (`#16 <https://github.com/ros2/teleop_twist_joy/issues/16>`_)
* Contributors: aravindsrj

2.2.2 (2019-10-23)
------------------
* Export interfaces for Shared Lib on Windows.
* Make teleop_twist_joy composable.
* Reenable cppcheck.
* Rename teleop_twist_joy.h to teleop_twist_joy.hpp
* Get some basic tests running (`#10 <https://github.com/ros2/teleop_twist_joy/issues/10>`_)
* Port config and launch to ROS 2 (`#11 <https://github.com/ros2/teleop_twist_joy/issues/11>`_)
* Contributors: Chris Lalancette, Scott K Logan, seanyen

2.2.0 (2019-05-31)
------------------
* Fix parameters so things actually work in Dashing. (`#9 <https://github.com/ros2/teleop_twist_joy/issues/9>`_)
* Contributors: Chris Lalancette

2.1.1 (2019-02-08)
------------------
* Add in the ability to control via parameters (`#8 <https://github.com/ros2/teleop_twist_joy/issues/8>`_)
* Contributors: Chris Lalancette

2.1.0 (2018-06-26)
------------------
* ParameterService auto started (`#7 <https://github.com/ros2/teleop_twist_joy/issues/7>`_)
* Contributors: Shane Loretz

2.0.0 (2017-12-08)
------------------
* Initial port to ROS2
* Contributors: Chris Lalancette, Mikael Arguedas, Deanna Hood

0.1.2 (2016-08-31)
------------------
* Fixed incorrect key. (`#21 <https://github.com/ros-teleop/teleop_twist_joy/issues/21>`__)
* Allow custom config file from location outside of this package
* Setting scale_angular_turbo if axis_angular is set so that turning works when turbo is pressed.
* Added turbo scale for angular velocities and accompanying test.
* Add LICENSE.txt.
* Contributors: Daniel Aden, Isaac I.Y. Saito, Mike Purvis, Tony Baltovski

0.1.1 (2015-06-27)
------------------
* Add rostests.
* Added maps to allow multi-dof velocity publishing.
* Added Xbox 360 controller example.
* Contributors: Mike Purvis, Tony Baltovski

0.1.0 (2014-07-25)
------------------
* Added configurations for Logitech Attack3 and Extreme 3D Pro joysticks.
* Initial version, with example config for PS3 joystick.
* Contributors: Mike Purvis, Tony Baltovski
