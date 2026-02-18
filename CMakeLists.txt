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
