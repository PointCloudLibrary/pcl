set(SUBSYS_NAME tools)
set(SUBSYS_DESC "Useful PCL-based command line tools")
set(SUBSYS_DEPS common io filters sample_consensus segmentation search kdtree features surface octree registration recognition geometry keypoints ml)
set(SUBSYS_OPT_DEPS vtk visualization)
set(DEFAULT ON)
set(REASON "")

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${SUBSYS_OPT_DEPS})

if(NOT build)
  return()
endif()

# to be filled with all targets the tools subsystem
set(PCL_TOOLS_ALL_TARGETS)

PCL_ADD_EXECUTABLE(pcl_sac_segmentation_plane COMPONENT ${SUBSYS_NAME} SOURCES sac_segmentation_plane.cpp)
target_link_libraries(pcl_sac_segmentation_plane pcl_common pcl_io pcl_sample_consensus pcl_segmentation)

PCL_ADD_EXECUTABLE(pcl_plane_projection COMPONENT ${SUBSYS_NAME} SOURCES plane_projection.cpp)
target_link_libraries (pcl_plane_projection pcl_common pcl_io pcl_sample_consensus)

PCL_ADD_EXECUTABLE(pcl_normal_estimation COMPONENT ${SUBSYS_NAME} SOURCES normal_estimation.cpp)
target_link_libraries (pcl_normal_estimation pcl_common pcl_io pcl_features pcl_kdtree)

PCL_ADD_EXECUTABLE(pcl_uniform_sampling COMPONENT ${SUBSYS_NAME} SOURCES uniform_sampling.cpp)
target_link_libraries (pcl_uniform_sampling pcl_common pcl_io pcl_filters pcl_keypoints pcl_kdtree)

PCL_ADD_EXECUTABLE(pcl_boundary_estimation COMPONENT ${SUBSYS_NAME} SOURCES boundary_estimation.cpp)
target_link_libraries (pcl_boundary_estimation pcl_common pcl_io pcl_features pcl_kdtree)

PCL_ADD_EXECUTABLE(pcl_cluster_extraction COMPONENT ${SUBSYS_NAME} SOURCES cluster_extraction.cpp)
target_link_libraries (pcl_cluster_extraction pcl_common pcl_io pcl_segmentation pcl_filters pcl_kdtree)

PCL_ADD_EXECUTABLE(pcl_fpfh_estimation COMPONENT ${SUBSYS_NAME} SOURCES fpfh_estimation.cpp)
target_link_libraries (pcl_fpfh_estimation pcl_common pcl_io pcl_features pcl_kdtree)

PCL_ADD_EXECUTABLE(pcl_pcd2ply COMPONENT ${SUBSYS_NAME} SOURCES pcd2ply.cpp)
target_link_libraries (pcl_pcd2ply pcl_common pcl_io)

PCL_ADD_EXECUTABLE(pcl_ply2pcd COMPONENT ${SUBSYS_NAME} SOURCES ply2pcd.cpp)
target_link_libraries (pcl_ply2pcd pcl_common pcl_io)

PCL_ADD_EXECUTABLE(pcl_xyz2pcd COMPONENT ${SUBSYS_NAME} SOURCES xyz2pcd.cpp)
target_link_libraries (pcl_xyz2pcd pcl_common pcl_io)

PCL_ADD_EXECUTABLE(pcl_pclzf2pcd COMPONENT ${SUBSYS_NAME} SOURCES pclzf2pcd.cpp)
target_link_libraries (pcl_pclzf2pcd pcl_common pcl_io)

PCL_ADD_EXECUTABLE(pcl_pcd2vtk COMPONENT ${SUBSYS_NAME} SOURCES pcd2vtk.cpp)
target_link_libraries (pcl_pcd2vtk pcl_common pcl_io)

PCL_ADD_EXECUTABLE(pcl_vfh_estimation COMPONENT ${SUBSYS_NAME} SOURCES vfh_estimation.cpp)
target_link_libraries (pcl_vfh_estimation pcl_common pcl_io pcl_features pcl_kdtree)

PCL_ADD_EXECUTABLE(pcl_spin_estimation COMPONENT ${SUBSYS_NAME} SOURCES spin_estimation.cpp)
target_link_libraries (pcl_spin_estimation pcl_common pcl_io pcl_features pcl_kdtree)

PCL_ADD_EXECUTABLE(pcl_voxel_grid COMPONENT ${SUBSYS_NAME} SOURCES voxel_grid.cpp)
target_link_libraries (pcl_voxel_grid pcl_common pcl_io pcl_filters)

PCL_ADD_EXECUTABLE(pcl_passthrough_filter COMPONENT ${SUBSYS_NAME} SOURCES passthrough_filter.cpp)
target_link_libraries (pcl_passthrough_filter pcl_common pcl_io pcl_filters)

PCL_ADD_EXECUTABLE(pcl_radius_filter COMPONENT ${SUBSYS_NAME} SOURCES radius_filter.cpp)
target_link_libraries (pcl_radius_filter pcl_common pcl_io pcl_filters)

PCL_ADD_EXECUTABLE(pcl_extract_feature COMPONENT ${SUBSYS_NAME} SOURCES extract_feature.cpp)
target_link_libraries (pcl_extract_feature pcl_common pcl_io pcl_features pcl_kdtree)

PCL_ADD_EXECUTABLE(pcl_compute_cloud_error COMPONENT ${SUBSYS_NAME} SOURCES compute_cloud_error.cpp)
target_link_libraries (pcl_compute_cloud_error pcl_common pcl_io pcl_kdtree pcl_search)

PCL_ADD_EXECUTABLE(pcl_train_unary_classifier COMPONENT ${SUBSYS_NAME} SOURCES train_unary_classifier.cpp)
target_link_libraries (pcl_train_unary_classifier pcl_common pcl_io pcl_segmentation)

PCL_ADD_EXECUTABLE(pcl_unary_classifier_segment COMPONENT ${SUBSYS_NAME} SOURCES unary_classifier_segment.cpp)
target_link_libraries (pcl_unary_classifier_segment pcl_common pcl_io pcl_segmentation)

PCL_ADD_EXECUTABLE(pcl_crf_segmentation COMPONENT ${SUBSYS_NAME} SOURCES crf_segmentation.cpp)
target_link_libraries (pcl_crf_segmentation pcl_common pcl_io pcl_segmentation)

PCL_ADD_EXECUTABLE(pcl_add_gaussian_noise COMPONENT ${SUBSYS_NAME} SOURCES add_gaussian_noise.cpp)
target_link_libraries (pcl_add_gaussian_noise pcl_common pcl_io)

PCL_ADD_EXECUTABLE(pcl_outlier_removal COMPONENT ${SUBSYS_NAME} SOURCES outlier_removal.cpp)
target_link_libraries (pcl_outlier_removal pcl_common pcl_io pcl_filters)

PCL_ADD_EXECUTABLE(pcl_mls_smoothing COMPONENT ${SUBSYS_NAME} SOURCES mls_smoothing.cpp)
target_link_libraries (pcl_mls_smoothing pcl_common pcl_io pcl_surface pcl_filters)

PCL_ADD_EXECUTABLE(pcl_marching_cubes_reconstruction COMPONENT ${SUBSYS_NAME} SOURCES marching_cubes_reconstruction.cpp)
target_link_libraries (pcl_marching_cubes_reconstruction pcl_common pcl_io pcl_surface)

PCL_ADD_EXECUTABLE(pcl_gp3_surface COMPONENT ${SUBSYS_NAME} SOURCES gp3_surface.cpp)
target_link_libraries (pcl_gp3_surface pcl_common pcl_io pcl_surface)

PCL_ADD_EXECUTABLE(pcl_icp COMPONENT ${SUBSYS_NAME} SOURCES icp.cpp)
target_link_libraries(pcl_icp pcl_common pcl_io pcl_registration)

PCL_ADD_EXECUTABLE(pcl_icp2d COMPONENT ${SUBSYS_NAME} SOURCES icp2d.cpp)
target_link_libraries(pcl_icp2d pcl_common pcl_io pcl_registration)

PCL_ADD_EXECUTABLE(pcl_elch COMPONENT ${SUBSYS_NAME} SOURCES elch.cpp)
target_link_libraries(pcl_elch pcl_common pcl_io pcl_registration)

PCL_ADD_EXECUTABLE(pcl_lum COMPONENT ${SUBSYS_NAME} SOURCES lum.cpp)
target_link_libraries(pcl_lum pcl_common pcl_io pcl_registration)

PCL_ADD_EXECUTABLE(pcl_ndt2d COMPONENT ${SUBSYS_NAME} SOURCES ndt2d.cpp)
target_link_libraries(pcl_ndt2d pcl_common pcl_io pcl_registration)

PCL_ADD_EXECUTABLE(pcl_ndt3d COMPONENT ${SUBSYS_NAME} SOURCES ndt3d.cpp)
target_link_libraries(pcl_ndt3d pcl_common pcl_io pcl_registration)

PCL_ADD_EXECUTABLE(pcl_pcd_change_viewpoint COMPONENT ${SUBSYS_NAME} SOURCES pcd_change_viewpoint.cpp)
target_link_libraries(pcl_pcd_change_viewpoint pcl_common pcl_io)

PCL_ADD_EXECUTABLE(pcl_concatenate_points_pcd COMPONENT ${SUBSYS_NAME} SOURCES concatenate_points_pcd.cpp)
target_link_libraries(pcl_concatenate_points_pcd pcl_common pcl_io)

PCL_ADD_EXECUTABLE(pcl_poisson_reconstruction COMPONENT ${SUBSYS_NAME} SOURCES poisson_reconstruction.cpp)
target_link_libraries(pcl_poisson_reconstruction pcl_common pcl_io pcl_surface)

PCL_ADD_EXECUTABLE(pcl_train_linemod_template COMPONENT ${SUBSYS_NAME} SOURCES train_linemod_template.cpp)
target_link_libraries(pcl_train_linemod_template pcl_common pcl_io pcl_segmentation pcl_recognition)

PCL_ADD_EXECUTABLE(pcl_match_linemod_template COMPONENT ${SUBSYS_NAME} SOURCES match_linemod_template.cpp)
target_link_libraries(pcl_match_linemod_template pcl_common pcl_io pcl_recognition)

PCL_ADD_EXECUTABLE(pcl_linemod_detection COMPONENT ${SUBSYS_NAME} SOURCES linemod_detection.cpp)
target_link_libraries(pcl_linemod_detection pcl_common pcl_io pcl_recognition)

PCL_ADD_EXECUTABLE(pcl_fast_bilateral_filter COMPONENT ${SUBSYS_NAME} SOURCES fast_bilateral_filter.cpp)
target_link_libraries(pcl_fast_bilateral_filter pcl_common pcl_io pcl_filters)

PCL_ADD_EXECUTABLE(pcl_demean_cloud COMPONENT ${SUBSYS_NAME} SOURCES demean_cloud.cpp)
target_link_libraries(pcl_demean_cloud pcl_common pcl_io)

PCL_ADD_EXECUTABLE(pcl_compute_hausdorff COMPONENT ${SUBSYS_NAME} SOURCES compute_hausdorff.cpp)
target_link_libraries(pcl_compute_hausdorff pcl_common pcl_io pcl_search)

PCL_ADD_EXECUTABLE(pcl_morph COMPONENT ${SUBSYS_NAME} SOURCES morph.cpp)
target_link_libraries(pcl_morph pcl_common pcl_io pcl_filters)

PCL_ADD_EXECUTABLE(pcl_progressive_morphological_filter COMPONENT ${SUBSYS_NAME} SOURCES progressive_morphological_filter.cpp)
target_link_libraries(pcl_progressive_morphological_filter pcl_common pcl_io pcl_filters pcl_segmentation)

PCL_ADD_EXECUTABLE(pcl_generate COMPONENT ${SUBSYS_NAME} SOURCES generate.cpp)
target_link_libraries(pcl_generate pcl_common pcl_io)

PCL_ADD_EXECUTABLE(pcl_local_max COMPONENT ${SUBSYS_NAME} SOURCES local_max.cpp)
target_link_libraries(pcl_local_max pcl_common pcl_io pcl_filters)

PCL_ADD_EXECUTABLE(pcl_grid_min COMPONENT ${SUBSYS_NAME} SOURCES grid_min.cpp)
target_link_libraries(pcl_grid_min pcl_common pcl_io pcl_filters)

if(WITH_OPENNI)
  PCL_ADD_EXECUTABLE(pcl_oni2pcd COMPONENT ${SUBSYS_NAME} SOURCES oni2pcd.cpp)
  target_link_libraries(pcl_oni2pcd pcl_common pcl_io)
endif()

if(QHULL_FOUND)
  PCL_ADD_EXECUTABLE(pcl_crop_to_hull COMPONENT ${SUBSYS_NAME} SOURCES crop_to_hull.cpp)
  target_link_libraries(pcl_crop_to_hull pcl_common pcl_io pcl_filters pcl_surface)

  PCL_ADD_EXECUTABLE(pcl_compute_hull COMPONENT ${SUBSYS_NAME} SOURCES compute_hull.cpp)
  target_link_libraries(pcl_compute_hull pcl_common pcl_io pcl_surface)
endif()

if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else()
  set(DEFAULT TRUE)
  set(REASON)
  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

  PCL_ADD_EXECUTABLE(pcl_png2pcd COMPONENT ${SUBSYS_NAME} SOURCES png2pcd.cpp)
  target_link_libraries(pcl_png2pcd pcl_common pcl_io)

  PCL_ADD_EXECUTABLE(pcl_pcd2png COMPONENT ${SUBSYS_NAME} SOURCES pcd2png.cpp)
  target_link_libraries(pcl_pcd2png pcl_common pcl_io)

  PCL_ADD_EXECUTABLE(pcl_tiff2pcd COMPONENT ${SUBSYS_NAME} SOURCES tiff2pcd.cpp)
  target_link_libraries(pcl_tiff2pcd pcl_common pcl_io)

  PCL_ADD_EXECUTABLE(pcl_ply2vtk COMPONENT ${SUBSYS_NAME} SOURCES ply2vtk.cpp)
  target_link_libraries(pcl_ply2vtk pcl_common pcl_io)

  PCL_ADD_EXECUTABLE(pcl_vtk2ply COMPONENT ${SUBSYS_NAME} SOURCES vtk2ply.cpp)
  target_link_libraries(pcl_vtk2ply pcl_common pcl_io)

  PCL_ADD_EXECUTABLE(pcl_vtk2obj COMPONENT ${SUBSYS_NAME} SOURCES vtk2obj.cpp)
  target_link_libraries(pcl_vtk2obj pcl_common pcl_io)

  PCL_ADD_EXECUTABLE(pcl_obj2vtk COMPONENT ${SUBSYS_NAME} SOURCES obj2vtk.cpp)
  target_link_libraries(pcl_obj2vtk pcl_common pcl_io)

  PCL_ADD_EXECUTABLE(pcl_obj2pcd COMPONENT ${SUBSYS_NAME} SOURCES obj2pcd.cpp)
  target_link_libraries(pcl_obj2pcd pcl_common pcl_io)

  if(${VTK_VERSION} VERSION_GREATER_EQUAL 9.0)
    target_link_libraries(pcl_obj2pcd VTK::FiltersCore)
  elseif(${VTK_VERSION} VERSION_GREATER_EQUAL 8.0)
    target_link_libraries(pcl_obj2pcd vtkFiltersCore)
  endif()

  PCL_ADD_EXECUTABLE(pcl_obj2ply COMPONENT ${SUBSYS_NAME} SOURCES obj2ply.cpp)
  target_link_libraries(pcl_obj2ply pcl_common pcl_io)

  PCL_ADD_EXECUTABLE(pcl_vtk2pcd COMPONENT ${SUBSYS_NAME} SOURCES vtk2pcd.cpp)
  target_link_libraries(pcl_vtk2pcd pcl_common pcl_io)

  if(${VTK_VERSION} VERSION_GREATER_EQUAL 9.0)
    target_link_libraries(pcl_vtk2pcd VTK::FiltersCore)
  elseif(${VTK_VERSION} VERSION_GREATER_EQUAL 8.0)
    target_link_libraries(pcl_vtk2pcd vtkFiltersCore)
  endif()

  if(BUILD_visualization)
    PCL_ADD_EXECUTABLE(pcl_obj_rec_ransac_model_opps COMPONENT ${SUBSYS_NAME} SOURCES obj_rec_ransac_model_opps.cpp)
    target_link_libraries(pcl_obj_rec_ransac_model_opps pcl_common pcl_visualization pcl_recognition)

    PCL_ADD_EXECUTABLE(pcl_obj_rec_ransac_hash_table COMPONENT ${SUBSYS_NAME} SOURCES obj_rec_ransac_hash_table.cpp)
    target_link_libraries(pcl_obj_rec_ransac_hash_table pcl_common pcl_visualization pcl_io pcl_recognition)

    PCL_ADD_EXECUTABLE(pcl_obj_rec_ransac_scene_opps COMPONENT ${SUBSYS_NAME} SOURCES obj_rec_ransac_scene_opps.cpp)
    target_link_libraries(pcl_obj_rec_ransac_scene_opps pcl_common pcl_visualization pcl_io pcl_recognition)

    PCL_ADD_EXECUTABLE(pcl_obj_rec_ransac_accepted_hypotheses COMPONENT ${SUBSYS_NAME} SOURCES obj_rec_ransac_accepted_hypotheses.cpp)
    target_link_libraries(pcl_obj_rec_ransac_accepted_hypotheses pcl_common pcl_visualization pcl_io pcl_recognition)

    PCL_ADD_EXECUTABLE(pcl_obj_rec_ransac_orr_octree COMPONENT ${SUBSYS_NAME} SOURCES obj_rec_ransac_orr_octree.cpp)
    target_link_libraries(pcl_obj_rec_ransac_orr_octree pcl_common pcl_visualization pcl_io pcl_recognition)

    PCL_ADD_EXECUTABLE(pcl_obj_rec_ransac_orr_octree_zprojection COMPONENT ${SUBSYS_NAME} SOURCES obj_rec_ransac_orr_octree_zprojection.cpp)
    target_link_libraries(pcl_obj_rec_ransac_orr_octree_zprojection pcl_common pcl_visualization pcl_io pcl_recognition)

    PCL_ADD_EXECUTABLE(pcl_obj_rec_ransac_result COMPONENT ${SUBSYS_NAME} SOURCES obj_rec_ransac_result.cpp)
    target_link_libraries(pcl_obj_rec_ransac_result pcl_common pcl_visualization pcl_io pcl_segmentation pcl_recognition)

    PCL_ADD_EXECUTABLE(pcl_registration_visualizer COMPONENT ${SUBSYS_NAME} SOURCES registration_visualizer.cpp)
    target_link_libraries(pcl_registration_visualizer pcl_common pcl_io pcl_kdtree pcl_filters pcl_registration pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_octree_viewer COMPONENT ${SUBSYS_NAME} SOURCES octree_viewer.cpp)
    target_link_libraries(pcl_octree_viewer pcl_common pcl_io pcl_octree pcl_visualization pcl_kdtree pcl_filters)

    PCL_ADD_EXECUTABLE(pcl_mesh2pcd COMPONENT ${SUBSYS_NAME} SOURCES mesh2pcd.cpp)
    target_link_libraries(pcl_mesh2pcd pcl_common pcl_io pcl_visualization pcl_filters)

    PCL_ADD_EXECUTABLE(pcl_mesh_sampling COMPONENT ${SUBSYS_NAME} SOURCES mesh_sampling.cpp)
    target_link_libraries(pcl_mesh_sampling pcl_common pcl_io pcl_visualization pcl_filters)

    PCL_ADD_EXECUTABLE(pcl_virtual_scanner COMPONENT ${SUBSYS_NAME} SOURCES virtual_scanner.cpp)
    target_link_libraries(pcl_virtual_scanner pcl_common pcl_io pcl_filters pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_voxel_grid_occlusion_estimation COMPONENT ${SUBSYS_NAME} SOURCES voxel_grid_occlusion_estimation.cpp)
    target_link_libraries (pcl_voxel_grid_occlusion_estimation pcl_common pcl_io pcl_filters pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_viewer COMPONENT ${SUBSYS_NAME} SOURCES pcd_viewer.cpp BUNDLE)
    target_link_libraries(pcl_viewer pcl_common pcl_io pcl_kdtree pcl_visualization pcl_search)

    PCL_ADD_EXECUTABLE(pcl_pcd_image_viewer COMPONENT ${SUBSYS_NAME} SOURCES image_viewer.cpp BUNDLE)
    target_link_libraries(pcl_pcd_image_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_timed_trigger_test COMPONENT ${SUBSYS_NAME} SOURCES timed_trigger_test.cpp)
    target_link_libraries(pcl_timed_trigger_test pcl_io pcl_common pcl_kdtree pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_hdl_viewer_simple COMPONENT ${SUBSYS_NAME} SOURCES hdl_viewer_simple.cpp)
    target_link_libraries(pcl_hdl_viewer_simple pcl_io pcl_common pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_vlp_viewer COMPONENT ${SUBSYS_NAME} SOURCES vlp_viewer.cpp)
    target_link_libraries(pcl_vlp_viewer pcl_io pcl_common pcl_visualization)

    if(WITH_OPENNI)
      PCL_ADD_EXECUTABLE(pcl_openni_save_image COMPONENT ${SUBSYS_NAME} SOURCES openni_save_image.cpp)
      target_link_libraries(pcl_openni_save_image pcl_common pcl_io pcl_visualization Boost::date_time)

      PCL_ADD_EXECUTABLE(pcl_pcd_grabber_viewer COMPONENT ${SUBSYS_NAME} SOURCES pcd_grabber_viewer.cpp BUNDLE)
      target_link_libraries(pcl_pcd_grabber_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)

      PCL_ADD_EXECUTABLE(pcl_image_grabber_saver COMPONENT ${SUBSYS_NAME} SOURCES image_grabber_saver.cpp BUNDLE)
      target_link_libraries(pcl_image_grabber_saver pcl_common pcl_io pcl_kdtree pcl_visualization)

      PCL_ADD_EXECUTABLE(pcl_image_grabber_viewer COMPONENT ${SUBSYS_NAME} SOURCES image_grabber_viewer.cpp BUNDLE)
      target_link_libraries(pcl_image_grabber_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)

      #PCL_ADD_EXECUTABLE(pcl_openni_viewer_simple COMPONENT ${SUBSYS_NAME} SOURCES openni_viewer_simple.cpp)
      #target_link_libraries(pcl_openni_viewer_simple pcl_common pcl_io pcl_kdtree pcl_visualization)

      PCL_ADD_EXECUTABLE(pcl_oni_viewer COMPONENT ${SUBSYS_NAME} SOURCES oni_viewer_simple.cpp BUNDLE)
      target_link_libraries(pcl_oni_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)

      PCL_ADD_EXECUTABLE(pcl_openni_viewer COMPONENT ${SUBSYS_NAME} SOURCES openni_viewer.cpp BUNDLE)
      target_link_libraries(pcl_openni_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)

      PCL_ADD_EXECUTABLE(pcl_openni_image COMPONENT ${SUBSYS_NAME} SOURCES openni_image.cpp BUNDLE)
      target_link_libraries(pcl_openni_image pcl_common pcl_io pcl_kdtree pcl_visualization Boost::date_time)
    endif()

    if(WITH_OPENNI2)
      PCL_ADD_EXECUTABLE(pcl_openni2_viewer COMPONENT ${SUBSYS_NAME} SOURCES openni2_viewer.cpp BUNDLE)
      target_link_libraries(pcl_openni2_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)
    endif()

    if(WITH_ENSENSO)
      PCL_ADD_EXECUTABLE(pcl_ensenso_viewer COMPONENT ${SUBSYS_NAME} SOURCES ensenso_viewer.cpp BUNDLE)
      target_link_libraries(pcl_ensenso_viewer pcl_common pcl_io pcl_visualization)
    endif()

    if(WITH_DAVIDSDK)
      PCL_ADD_EXECUTABLE(pcl_davidsdk_viewer COMPONENT ${SUBSYS_NAME} SOURCES davidsdk_viewer.cpp BUNDLE)
      target_link_libraries(pcl_davidsdk_viewer pcl_common pcl_io pcl_visualization)
    endif()

    if(WITH_DSSDK)
      PCL_ADD_EXECUTABLE(pcl_depth_sense_viewer COMPONENT ${SUBSYS_NAME} SOURCES depth_sense_viewer.cpp BUNDLE)
      target_link_libraries(pcl_depth_sense_viewer pcl_common pcl_io pcl_visualization)
    endif()

    if(WITH_RSSDK)
      PCL_ADD_EXECUTABLE(pcl_real_sense_viewer COMPONENT ${SUBSYS_NAME} SOURCES real_sense_viewer.cpp BUNDLE)
      target_link_libraries(pcl_real_sense_viewer pcl_common pcl_io pcl_visualization)
    endif()
  endif()
endif()

PCL_ADD_EXECUTABLE(pcl_transform_point_cloud COMPONENT ${SUBSYS_NAME} SOURCES transform_point_cloud.cpp)
target_link_libraries (pcl_transform_point_cloud pcl_common pcl_io pcl_registration)

PCL_ADD_EXECUTABLE(pcl_transform_from_viewpoint COMPONENT ${SUBSYS_NAME} SOURCES transform_from_viewpoint.cpp)
target_link_libraries (pcl_transform_from_viewpoint pcl_common pcl_io pcl_registration)

find_package(tide QUIET)
if(Tide_FOUND)
  include_directories(SYSTEM ${Tide_INCLUDE_DIRS})
  add_definitions(${Tide_DEFINITIONS})
  PCL_ADD_EXECUTABLE(pcl_video COMPONENT ${SUBSYS_NAME} SOURCES pcl_video.cpp)
  target_link_libraries(pcl_video pcl_common pcl_io pcl_visualization
    ${Tide_LIBRARIES})
endif()

if(CMAKE_GENERATOR_IS_IDE)
  set(SUBSYS_TARGET_NAME TOOLS_BUILD)
else()
  set(SUBSYS_TARGET_NAME ${SUBSYS_NAME})
endif()
add_custom_target(${SUBSYS_TARGET_NAME} DEPENDS ${PCL_TOOLS_ALL_TARGETS})
set_target_properties(${SUBSYS_TARGET_NAME} PROPERTIES FOLDER "Tools")

