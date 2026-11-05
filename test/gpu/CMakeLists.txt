if(NOT HAVE_CUDA)
  return()
endif()

add_subdirectory(filters)
add_subdirectory(octree)
