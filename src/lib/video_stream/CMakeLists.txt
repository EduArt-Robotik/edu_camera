message(STATUS "building video_stream module.")
add_library(video_stream SHARED
  video_stream_server.cpp
  video_stream_client.cpp
)

target_include_directories(video_stream PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(video_stream
  PUBLIC
    ${OpenCV_LIBRARIES}
)

list(APPEND BUILT_LIBRARIES video_stream)

# Populating Built Libraries
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)
