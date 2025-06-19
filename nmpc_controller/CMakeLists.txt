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

add_library(nmpc_controller
  src/gen/eval_g_spirit.cpp
  src/gen/eval_jac_g_spirit.cpp
  src/gen/eval_hess_g_spirit.cpp
  src/gen/eval_g_a1.cpp
  src/gen/eval_jac_g_a1.cpp
  src/gen/eval_hess_g_a1.cpp
  src/gen/eval_g_leg_simple.cpp
  src/gen/eval_jac_g_leg_simple.cpp
  src/gen/eval_hess_g_leg_simple.cpp
  src/gen/eval_g_leg_complex.cpp
  src/gen/eval_jac_g_leg_complex.cpp
  src/gen/eval_hess_g_leg_complex.cpp
  src/gen/eval_g_leg_simple_to_complex.cpp
  src/gen/eval_jac_g_leg_simple_to_complex.cpp
  src/gen/eval_hess_g_leg_simple_to_complex.cpp
  src/gen/eval_g_leg_complex_to_simple.cpp
  src/gen/eval_jac_g_leg_complex_to_simple.cpp
  src/gen/eval_hess_g_leg_complex_to_simple.cpp
  src/quad_nlp_utils.cpp
  src/quad_nlp.cpp
  src/quad_nlp_utils.cpp
  src/adaptive_complexity_utils.cpp
  src/nmpc_controller.cpp
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
