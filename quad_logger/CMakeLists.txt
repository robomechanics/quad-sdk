cmake_minimum_required(VERSION 3.5)

project(quad_logger)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)

install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}

)

ament_package()
