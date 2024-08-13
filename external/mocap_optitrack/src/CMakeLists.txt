
add_executable(${PROJECT_NAME}_mocap_node
  natnet/natnet_packet_definition.cpp
  natnet/natnet_messages.cpp
  socket.cpp
  version.cpp
  mocap_node.cpp
  mocap_config.cpp
  data_model.cpp
  rigid_body_publisher.cpp
)

target_link_libraries(${PROJECT_NAME}_mocap_node 
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES})

set_target_properties(${PROJECT_NAME}_mocap_node PROPERTIES
                      OUTPUT_NAME mocap_node PREFIX "")
#target_compile_definitions(
#  ${PROJECT_NAME}_mocap_node PRIVATE
#  RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_FATAL
#)

install(
  TARGETS ${PROJECT_NAME}_mocap_node
  DESTINATION lib/${PROJECT_NAME}
)
