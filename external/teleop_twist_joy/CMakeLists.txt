cmake_minimum_required(VERSION 2.8.3)
project(teleop_twist_joy)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/teleop_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_node
  PROPERTIES OUTPUT_NAME teleop_node PREFIX "")

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp(include/ src/)
  roslint_python()
  roslint_add_test()

  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/teleop.launch)

  find_package(rostest REQUIRED)
  include_directories(include ${catkin_INCLUDE_DIRS})

  # Check axes and scaling.
  add_rostest(test/differential_joy.test)
  add_rostest(test/holonomic_joy.test)
  add_rostest(test/six_dof_joy.test)

  # Check enable and turbo button logic.
  add_rostest(test/no_enable_joy.test)
  add_rostest(test/turbo_enable_joy.test)
  add_rostest(test/only_turbo_joy.test)
  add_rostest(test/turbo_angular_enable_joy.test)
  add_rostest(test/turbo_angular_enable_joy_with_rosparam_map.test)
endif()
