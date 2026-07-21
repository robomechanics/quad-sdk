cmake_minimum_required(VERSION 3.8)
project(nmpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(quad_utils REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include 
  /usr/local/include/coin-or
)

link_directories(/usr/local/lib)

# Collect generated CasADi source files
file(GLOB NMPC_GEN_SOURCES src/gen/*.cpp)

add_library(nmpc_controller
  ${NMPC_GEN_SOURCES}
  src/quad_nlp_utils.cpp
  src/quad_nlp.cpp
  src/adaptive_complexity_utils.cpp
  src/nmpc_controller.cpp
)

# These are machine-emitted straight-line math, not hand-edited, so silence
# their warnings (unused params, VLAs) to keep the build output readable.
# In optimized builds also cap them at -O1 to reduce compiler RAM usage:
# -O1 is nearly as fast as -O2 at runtime but uses far less memory during
# compilation (the hessian/jacobian files are 20-40K lines each).
set(NMPC_GEN_COMPILE_OPTIONS "-w")
if(CMAKE_BUILD_TYPE MATCHES "Release|RelWithDebInfo")
  list(APPEND NMPC_GEN_COMPILE_OPTIONS "-O1")
endif()
set_source_files_properties(${NMPC_GEN_SOURCES}
  PROPERTIES COMPILE_OPTIONS "${NMPC_GEN_COMPILE_OPTIONS}"
)

target_link_libraries(nmpc_controller
  Eigen3::Eigen
  ipopt
)

ament_target_dependencies(nmpc_controller
  rclcpp
  std_msgs
  quad_msgs
  quad_utils
  grid_map_core
  grid_map_ros
  tf2_eigen
  tf2_geometry_msgs
)

# Export include directories
target_include_directories(nmpc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)


install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS nmpc_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(Eigen3 tf2_eigen)
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test
    test/test_nmpc_controller.cpp
    test/test_quad_nlp.cpp
  )
  if(TARGET ${PROJECT_NAME}_test)
    target_compile_definitions(${PROJECT_NAME}_test PRIVATE
      NMPC_CONTROLLER_TESTING
    )
    target_link_libraries(${PROJECT_NAME}_test nmpc_controller)
    target_include_directories(${PROJECT_NAME}_test PRIVATE
      include
      ${EIGEN3_INCLUDE_DIRS}
    )
    ament_target_dependencies(${PROJECT_NAME}_test
      rclcpp
      std_msgs
      quad_msgs
      quad_utils
      grid_map_core
      grid_map_ros
      grid_map_msgs
      tf2_eigen
      tf2_geometry_msgs
    )
    set_tests_properties(${PROJECT_NAME}_test PROPERTIES
      ENVIRONMENT
        "NMPC_TEST_PARAMS=${CMAKE_CURRENT_SOURCE_DIR}/config/nmpc_controller.yaml;NMPC_LOCAL_PLANNER_PARAMS=${CMAKE_CURRENT_SOURCE_DIR}/../local_planner/config/local_planner.yaml;NMPC_ROBOT_PARAMS=${CMAKE_CURRENT_SOURCE_DIR}/../quad_utils/config/spirit.yaml;NMPC_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/.."
    )
  endif()

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
endif()

ament_package()
