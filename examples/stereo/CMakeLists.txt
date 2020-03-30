if(NOT BUILD_visualization)
  return()
endif()

PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS visualization)

## Find VTK
if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else()
  set(DEFAULT TRUE)
  set(REASON)
  include(${VTK_USE_FILE})
endif()

PCL_ADD_EXAMPLE(pcl_example_stereo_baseline FILES example_stereo_baseline.cpp
                LINK_WITH pcl_common pcl_visualization pcl_stereo pcl_io)
