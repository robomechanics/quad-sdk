cmake_minimum_required(VERSION 3.8)
project(isaac_plugins)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE
)

# Also register the wrapper under lib/<pkg> so `ros2 run isaac_plugins
# run_isaac_bridge.sh` resolves. ros2 run only scans lib/<pkg>, not
# share/<pkg>/scripts.
install(
  PROGRAMS scripts/run_isaac_bridge.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
