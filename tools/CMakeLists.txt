set (SUBSYS_NAME tools)
set (SUBSYS_DESC "Useful PCL-based command line tools")
set (SUBSYS_DEPS common io filters sample_consensus segmentation kdtree features surface octree registration visualization)
set (DEFAULT ON)
set (REASON "")

PCL_SUBSYS_OPTION (build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND (build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if (build)

  PCL_ADD_EXECUTABLE (plane_projection ${SUBSYS_NAME} plane_projection.cpp)
  target_link_libraries (plane_projection pcl_common pcl_io pcl_sample_consensus)

  PCL_ADD_EXECUTABLE (normal_estimation ${SUBSYS_NAME} normal_estimation.cpp)
  target_link_libraries (normal_estimation pcl_common pcl_io pcl_features pcl_kdtree)

  PCL_ADD_EXECUTABLE (boundary_estimation ${SUBSYS_NAME} boundary_estimation.cpp)
  target_link_libraries (boundary_estimation pcl_common pcl_io pcl_features pcl_kdtree)

  PCL_ADD_EXECUTABLE (cluster_extraction ${SUBSYS_NAME} cluster_extraction.cpp)
  target_link_libraries (cluster_extraction pcl_common pcl_io pcl_segmentation pcl_filters pcl_kdtree)

  PCL_ADD_EXECUTABLE (fpfh_estimation ${SUBSYS_NAME} fpfh_estimation.cpp)
  target_link_libraries (fpfh_estimation pcl_common pcl_io pcl_features pcl_kdtree)

  PCL_ADD_EXECUTABLE (vfh_estimation ${SUBSYS_NAME} vfh_estimation.cpp)
  target_link_libraries (vfh_estimation pcl_common pcl_io pcl_features pcl_kdtree)
  
  PCL_ADD_EXECUTABLE (spin_estimation ${SUBSYS_NAME} spin_estimation.cpp)
  target_link_libraries (spin_estimation pcl_common pcl_io pcl_features pcl_kdtree)

  PCL_ADD_EXECUTABLE (voxel_grid ${SUBSYS_NAME} voxel_grid.cpp)
  target_link_libraries (voxel_grid pcl_common pcl_io pcl_filters)
	
  PCL_ADD_EXECUTABLE (passthrough_filter ${SUBSYS_NAME} passthrough_filter.cpp)
  target_link_libraries (passthrough_filter pcl_common pcl_io pcl_filters)
	
  PCL_ADD_EXECUTABLE (extract_feature ${SUBSYS_NAME} extract_feature.cpp)
  target_link_libraries (extract_feature pcl_common pcl_io pcl_features pcl_kdtree)
  
  PCL_ADD_EXECUTABLE (compute_cloud_error ${SUBSYS_NAME} compute_cloud_error.cpp)
  target_link_libraries (compute_cloud_error pcl_common pcl_io pcl_kdtree)
  
  PCL_ADD_EXECUTABLE (add_gaussian_noise ${SUBSYS_NAME} add_gaussian_noise.cpp)
  target_link_libraries (add_gaussian_noise pcl_common pcl_io)
  
  PCL_ADD_EXECUTABLE (mls_smoothing ${SUBSYS_NAME} mls_smoothing.cpp)
  target_link_libraries (mls_smoothing pcl_common pcl_io pcl_surface)

  PCL_ADD_EXECUTABLE (gp3_surface ${SUBSYS_NAME} gp3_surface.cpp)
  target_link_libraries (gp3_surface pcl_common pcl_io pcl_surface)
  
  if(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
  else(NOT VTK_FOUND)
      set(DEFAULT TRUE)
      set(REASON)
      set(VTK_USE_FILE ${VTK_USE_FILE} CACHE INTERNAL "VTK_USE_FILE")
      include (${VTK_USE_FILE})
      include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
      
      PCL_ADD_EXECUTABLE(registration_visualizer ${SUBSYS_NAME} registration_visualizer.cpp)
      target_link_libraries(registration_visualizer pcl_common pcl_io pcl_kdtree pcl_filters pcl_registration pcl_visualization)

      PCL_ADD_EXECUTABLE(octree_viewer ${SUBSYS_NAME} octree_viewer.cpp)
      target_link_libraries(octree_viewer pcl_common pcl_io pcl_octree pcl_visualization pcl_kdtree)   

      PCL_ADD_EXECUTABLE(mesh2pcd ${SUBSYS_NAME} mesh2pcd.cpp)
      target_link_libraries(mesh2pcd pcl_common pcl_io pcl_visualization pcl_filters vtkCommon)
      
      PCL_ADD_EXECUTABLE(mesh_sampling ${SUBSYS_NAME} mesh_sampling.cpp)
      target_link_libraries(mesh_sampling pcl_common pcl_io pcl_visualization pcl_filters vtkCommon)
  endif(NOT VTK_FOUND)

  PCL_ADD_EXECUTABLE (transform_point_cloud ${SUBSYS_NAME} transform_point_cloud.cpp)
  target_link_libraries (transform_point_cloud pcl_common pcl_io pcl_registration)

endif ()
