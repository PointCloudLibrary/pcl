set(SUBSYS_NAME cuda_apps)
set(SUBSYS_DESC "Point cloud library CUDA apps")
set(SUBSYS_DEPS cuda cuda_common cuda_io cuda_features common io visualization)

# ---[ Point Cloud Library - pcl/cuda/apps

# Find VTK 
if(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
elseif(NOT wxWidgets_FOUND)
    set(DEFAULT TRUE)   # wxWidgets is optional
    set(REASON "wxWidgets was not found.")
    include (${VTK_USE_FILE})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
else(NOT VTK_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
    message(STATUS "VTK found (include: ${VTK_INCLUDE_DIRS}, lib: ${VTK_LIBRARY_DIRS})")
    include (${VTK_USE_FILE})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
endif(NOT VTK_FOUND)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
mark_as_advanced("BUILD_${SUBSYS_NAME}")
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
  if(VTK_FOUND)

    PCL_CUDA_ADD_EXECUTABLE(kinect_viewer_cuda ${SUBSYS_NAME} src/kinect_viewer_cuda.cpp)
    target_link_libraries (kinect_viewer_cuda pcl_io pcl_cuda_io pcl_visualization pcl_common pcl_kdtree pcl_range_image)

    PCL_CUDA_ADD_EXECUTABLE(kinect_normals_cuda ${SUBSYS_NAME} src/kinect_normals_cuda.cpp)
    target_link_libraries (kinect_normals_cuda pcl_io pcl_cuda_io pcl_cuda_features pcl_visualization pcl_common pcl_kdtree pcl_range_image)

  endif(VTK_FOUND)
endif(build)




