if(NOT BUILD_outofcore)
  return()
endif()

# Find VTK
if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else()
  set(DEFAULT TRUE)
  set(REASON)
endif()

PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS outofcore io common octree filters visualization EXT_DEPS vtk)
PCL_ADD_EXAMPLE (pcl_example_outofcore_with_lod FILES example_outofcore_with_lod.cpp LINK_WITH pcl_outofcore pcl_common pcl_io pcl_octree pcl_filters)
PCL_ADD_EXAMPLE (pcl_example_outofcore FILES example_outofcore.cpp LINK_WITH pcl_outofcore pcl_common pcl_io pcl_octree pcl_filters)
