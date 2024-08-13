^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mocap_optitrack
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-10-26)
------------------
* Feat: implement ros1 features and fixes
* Removed wrong and undesired transformation
* Implemented optitrack timestamp
  The message timestamp will now be the time at which optitrack acquired the data instead of the time at which the ROS node received the data
* Fixed deprecated parameters
* Allowed undeclared parameters
* Port to ROS2
* Natnet and server version can now be set in config file
  Added a way to set both natnet and server version from yaml file, so you don't have to restart the broadcast each time you launch the mocap node. Add "version: [3,0,0,0]" in the optitrack_config-part of the config file
* Updates all source files with BSD license blurb.
  Also removes std_msgs has a build dependency as its not a dependency in
  the code anywhere. Cleans up package.xml to format 2, adds all authors, and
  corrects maintainer.
* Adds 2nd body to example config file
  Also removes use_new_coordinates from the example config file. This is now calculated internally from the server version.
* Removes console output from launch file
* Completes refactoring. Tested with NatNet version 3.
* Finishes a majority of the refactoring and verifies it works for NatNet version 3.0
* Heavily refactors code to improve compatibility with NatNet SDK and improve single responsibility.
* Debugging NatNet version 3.0..
* Added version checking to allow for support for changes related to
  certain NatNet versions.
* Added support for new (Motive 1.7+) and old coordinate system.
* updated for recent natnet protocol
* Removed un-needed code.
* Turned down verbosity for less spam when output=screen.
* mocap configuration file is an option to launch file.
* Optitrack multicast address is now a parameter in the config file.
* Fixed config parameter name in example configuration file.
  frame_id -> child_frame_id: The latter is used in mocap_config.cpp (CHILD_FRAME_ID_PARAM_NAME)
* Removed tabs from launch file.
* Switched to a pose stamped message.  Added parameters for parent and child frames.
* initial2
* initial
* Update socket.h
  fixed a typo
* Install the config directory as well, and add a roslaunch file check macro for the launch file to prevent a regression.
* Updated package.xml and cleaned up CMakeLists.txt
* Split off src portion of CMakeLists.
* Lowercase filenames for socket, remove separate exception header, move headers into package subdirectory.
* Tested with actual system and deleted unneeded old files.
* Tried to convert to catkin.
* Tried to convert to catkin.
* Updated copyright & authors in manifest.xml.
* Cleaned up sample configuration file.
* Updated supported NatNet protocol and description.
* Added missing stack dependencies.
* Converted to a unary stack.
  Added stack.xml to make this a unary stack. Also fixed missing include
  for ROS_DEBUG.
* Squashing for release
* Some commenting to yaml config file
* Fixed pose streaming for NaN values.
* Tweaking configs
  Committer: Ryan Gariepy <rgariepy@clearpathrobotics.com>
  On branch hsi-upgrades
  Changes to be committed:
  (use "git reset HEAD <file>..." to unstage)
  modified:   mocap_optitrack/config/mocap.yaml
  modified:   mocap_optitrack/src/mocap_config.cpp
* Fixed inversion in published TF.
* Renamed parameters to reflect the fact that tf will always be published to the same topic.
* Fixed loop and termination code to handle SIGINT correctly.
* Added some headers & comments.
* Fix conversion from OptiTrack to ROS coordinate system.
* Check for all zeros in data. This occurs when the object is not
  being tracked by Optitrack. Zero data should not result in any
  pose/tf being published in ROS.
* Simplified conversion from Optitrack to Pose2D.
* Cleanup and fixed uninitialized variable problems. Node now works
  consistently, since variables are initialized as expected.
* Some cleanup and prepare for correct mocap->ROS data conversion.
* Implemented multi-object support. Each object may be exposed as a
  tf, Pose, or Pose2D with different topics configurable for each
  rigid body & datatype.
* Fixed deserialization of Pose values. Code was incorrectly trying
  to read position/orientation as float64 values instead of the actual
  float that's in the packet.
* Fixed remaining packet parsing bugs. Can now read multiple data sets from NatNet packets.
* Cleaned up parsing a bit, but getting some garbage in output.
* Fixed a bunch of stuff in OptiTrack packet parsing: crashes, memory leaks, some ugliness. Added Pose and Pose2D topics for using packages that may require these kinds of inputs. Added three parameters to enable/disable each of these topics: publish_transform, publish_pose, and publish_ground_pose.
* Fixed memory allocation and parsing.
* Don't launch the silly xterm.
* git-svn-id: http://ais-bonn-ros-pkg.googlecode.com/svn/trunk/stacks/ais_bonn_drivers@31 ee974301-c962-0d43-73a8-0c9e5eb6d2a8
* Contributors: 321thijs123, Administrator, Alex Bencz, Felix Duvallet, Hyon Lim, JD Yamokoski, Mike Purvis, Ryan Gariepy, Tony Baltovski, Tønnes Nygaard, Yannick de Hoop, Ziyang LI, codenotes, joerg.stueckler.bw@gmail.com, user, vinnamkim
