cmake_minimum_required(VERSION 3.8)
project(gazebo_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-transport11 REQUIRED)
# find_library(Protobuf REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  config/quad_control.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# Link Targets to IGN Transport Libraries
# target_link_libraries(ContactStatePublisher
#   ignition-transport11::ignition-transport
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
