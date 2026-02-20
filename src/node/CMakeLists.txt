add_executable(camera_node
  camera_node.cpp
)
target_include_directories(camera_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(camera_node
  video_stream
  camera
)
ament_target_dependencies(camera_node
  "rclcpp"
)

list(APPEND BUILT_NODES camera_node)

# Populating Built Nodes
set(BUILT_NODES ${BUILT_NODES} PARENT_SCOPE)
