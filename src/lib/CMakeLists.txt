add_subdirectory(video_stream)

# Propagate BUILT_LIBRARIES to parent scope
set(BUILT_LIBRARIES ${BUILT_LIBRARIES} PARENT_SCOPE)
