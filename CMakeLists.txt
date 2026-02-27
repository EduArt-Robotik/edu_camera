cmake_minimum_required(VERSION 3.8)
project(edu_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rosidl_default_generators REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)

## Build Defined ROS Messages
rosidl_generate_interfaces(${PROJECT_NAME}
# Messages
# Services
  "srv/SubscribeToStream.srv"
#  DEPENDENCIES
)
# Create a target that can be used by all buildiung targets when the custom message are required for them.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Building Libraries
add_subdirectory(src/lib)

# Building Nodes
add_subdirectory(src/node)

# Installing Content
install(
  DIRECTORY include/
  DESTINATION include
)
# install(
#   DIRECTORY
#     launch
#     parameter
#   DESTINATION share/${PROJECT_NAME}
# )

message(STATUS "Nodes: ${BUILT_NODES}")
install(
  TARGETS
    ${BUILT_NODES}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  # for humble...
  # RUNTIME DESTINATION bin
  # for galactic...
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

message(STATUS "Libraries: ${BUILT_LIBRARIES}")
install(
  TARGETS
    ${BUILT_LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Ament Package Export
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${BUILT_LIBRARIES}
)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_package()
