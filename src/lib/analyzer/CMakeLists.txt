message(STATUS "building analyzer module.")
add_library(analyzer SHARED
  wifi_metric.cpp
  wifi_quality_monitor.cpp
  metric_rssi.cpp
  metric_snr.cpp
  metric_link_quality.cpp
  metric_latency.cpp
  metric_packet_loss.cpp
)
target_include_directories(analyzer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
    ${GSTREAMER_VIDEO_INCLUDE_DIRS}
)
target_link_libraries(analyzer
    ${OpenCV_LIBRARIES}
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
    ${GSTREAMER_VIDEO_LIBRARIES}
)
ament_target_dependencies(analyzer
  "rclcpp"
)

list(APPEND BUILT_LIBRARIES analyzer)

# Populating Built Libraries
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)
