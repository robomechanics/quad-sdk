
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

target_link_libraries(${PROJECT_NAME}_mocap_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_mocap_node PROPERTIES
                      OUTPUT_NAME mocap_node PREFIX "")

install(TARGETS 
  ${PROJECT_NAME}_mocap_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
