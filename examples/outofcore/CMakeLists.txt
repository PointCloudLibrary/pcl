if (BUILD_outofcore)

  # Find VTK
  if(NOT VTK_FOUND)
      set(DEFAULT FALSE)
      set(REASON "VTK was not found.")
  else(NOT VTK_FOUND)
      set(DEFAULT TRUE)
      set(REASON)
      include (${VTK_USE_FILE})
  endif(NOT VTK_FOUND)

  PCL_SUBSYS_DEPEND (build ${SUBSYS_NAME} DEPS outofcore io common octree filters visualization EXT_DEPS vtk)
  PCL_ADD_EXAMPLE (pcl_example_outofcore_with_lod FILES example_outofcore_with_lod.cpp LINK_WITH pcl_outofcore pcl_common pcl_io pcl_octree pcl_filters)
  PCL_ADD_EXAMPLE (pcl_example_outofcore FILES example_outofcore.cpp LINK_WITH pcl_outofcore pcl_common pcl_io pcl_octree pcl_filters)

endif (BUILD_outofcore)
