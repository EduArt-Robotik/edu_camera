add_subdirectory(video_stream)
add_subdirectory(camera)
add_subdirectory(analyzer)

# Propagate BUILT_LIBRARIES to parent scope
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)
