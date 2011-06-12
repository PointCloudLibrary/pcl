set(SUBSYS_NAME apps)
set(SUBSYS_DESC "Application examples/samples that show how PCL works")
set(SUBSYS_DEPS common io filters sample_consensus segmentation visualization kdtree features surface)

if(NOT wxWidgets_FOUND)
  set(DEFAULT TRUE)   # wxWidgets is optional
  set(REASON "wxWidgets was not found.")
else(NOT wxWidgets_FOUND)
  include(${wxWidgets_USE_FILE})
  include_directories(${wxWidgets_INCLUDE_DIRS})
endif(NOT wxWidgets_FOUND)

# Find VTK 
if(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
    include (${VTK_USE_FILE})
endif(NOT VTK_FOUND)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
  if (VTK_FOUND)
    #PCL_ADD_EXECUTABLE(surflet_feature_object_recognition ${SUBSYS_NAME} src/surflet_feature_object_recognition.cpp)
    #target_link_libraries(surflet_feature_object_recognition pcl_common pcl_io pcl_filters pcl_features pcl_visualization)
    
	PCL_ADD_EXECUTABLE(openni_voxel_grid ${SUBSYS_NAME} src/openni_voxel_grid.cpp)
	target_link_libraries(openni_voxel_grid pcl_common pcl_io pcl_filters pcl_visualization)

	PCL_ADD_EXECUTABLE(openni_passthrough ${SUBSYS_NAME} src/openni_passthrough.cpp)
	target_link_libraries(openni_passthrough pcl_common pcl_io pcl_filters pcl_visualization)

	PCL_ADD_EXECUTABLE(openni_planar_segmentation ${SUBSYS_NAME} src/openni_planar_segmentation.cpp)
	target_link_libraries(openni_planar_segmentation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus)

  #    PCL_ADD_EXECUTABLE(openni_ii_normal_estimation ${SUBSYS_NAME} src/openni_ii_normal_estimation.cpp)
  #  target_link_libraries(openni_ii_normal_estimation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features)

  #  PCL_ADD_EXECUTABLE(openni_planar_convex_hull ${SUBSYS_NAME} src/openni_planar_convex_hull.cpp)
  #  target_link_libraries(openni_planar_convex_hull pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_surface)

    #    PCL_ADD_EXECUTABLE(openni_range_image_viewer ${SUBSYS_NAME} src/openni_range_image_viewer.cpp)
    #    target_link_libraries(openni_range_image_viewer pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_surface)


  endif()
endif(build)
