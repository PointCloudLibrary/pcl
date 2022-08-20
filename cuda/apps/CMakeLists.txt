set(SUBSYS_NAME cuda_apps)
set(SUBSYS_DESC "Point cloud library CUDA apps")
set(SUBSYS_DEPS cuda_common cuda_io cuda_features cuda_segmentation cuda_sample_consensus common io visualization geometry)

# ---[ Point Cloud Library - pcl/cuda/apps
# Find VTK
if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else()
  set(DEFAULT TRUE)
  set(REASON)
  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
mark_as_advanced("BUILD_${SUBSYS_NAME}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

if(NOT build)
  return()
endif()

if(VTK_FOUND AND WITH_OPENNI)
  PCL_CUDA_ADD_EXECUTABLE(kinect_viewer_cuda COMPONENT ${SUBSYS_NAME} SOURCES src/kinect_viewer_cuda.cpp)
  target_link_libraries (kinect_viewer_cuda pcl_io pcl_cuda_io pcl_visualization pcl_common pcl_kdtree)
endif()
