# Camera Node
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


# Wifi Network Analyzer Node
add_executable(wifi_network_analyzer_node
  wifi_network_analyzer_node.cpp
)
target_include_directories(wifi_network_analyzer_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(wifi_network_analyzer_node
  analyzer
)
ament_target_dependencies(wifi_network_analyzer_node
  "rclcpp"
  "std_msgs"
)

list(APPEND BUILT_NODES wifi_network_analyzer_node)


# Stream Client Node
add_executable(stream_client_node
  stream_client_node.cpp
)
target_include_directories(stream_client_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(stream_client_node
  video_stream
)
ament_target_dependencies(stream_client_node
  "rclcpp"
)

list(APPEND BUILT_NODES stream_client_node)

# Populating Built Nodes
set(BUILT_NODES ${BUILT_NODES} PARENT_SCOPE)
