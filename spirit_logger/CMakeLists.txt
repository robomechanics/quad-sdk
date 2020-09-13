cmake_minimum_required(VERSION 2.8.3)
project(spirit_logger)

find_package(catkin REQUIRED COMPONENTS rospy tf)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS rospy tf
  INCLUDE_DIRS
  LIBRARIES
)

catkin_install_python(PROGRAMS scripts/bag_reader.py scripts/read_bag.py scripts/mouse_interface.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
