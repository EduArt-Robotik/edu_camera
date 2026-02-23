message(STATUS "building video_stream module.")
add_library(video_stream SHARED
  video_stream_server.cpp
  video_stream_client.cpp
  video_gstream.cpp
  gstream_pipeline.cpp
)
target_include_directories(video_stream
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
    ${GSTREAMER_VIDEO_INCLUDE_DIRS}
)
target_link_libraries(video_stream
    ${OpenCV_LIBRARIES}
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    ${GSTREAMER_VIDEO_LIBRARIES}
)
ament_target_dependencies(video_stream
  "rclcpp"
)

list(APPEND BUILT_LIBRARIES video_stream)

# Populating Built Libraries
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)
