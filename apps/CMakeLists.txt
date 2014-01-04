set(SUBSYS_NAME apps)
set(SUBSYS_DESC "Application examples/samples that show how PCL works")
set(SUBSYS_DEPS common geometry io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search recognition ml stereo)

# Find VTK
if(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
    include("${VTK_USE_FILE}")
endif(NOT VTK_FOUND)

# OpenNI found?
if(NOT OPENNI_FOUND)
    set(DEFAULT FALSE)
    set(REASON "OpenNI was not found.")
else(NOT OPENNI_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
endif(NOT OPENNI_FOUND)

set(DEFAULT FALSE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} OPT_DEPS openni vtk)

if(build)

  include_directories("${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/include")

  PCL_ADD_EXECUTABLE(pcl_test_search_speed "${SUBSYS_NAME}" src/test_search.cpp)
  target_link_libraries(pcl_test_search_speed pcl_common pcl_io pcl_search pcl_kdtree pcl_visualization)

  PCL_ADD_EXECUTABLE(pcl_nn_classification_example "${SUBSYS_NAME}" src/nn_classification_example.cpp)
  target_link_libraries(pcl_nn_classification_example pcl_common pcl_io pcl_features pcl_kdtree)

  PCL_ADD_EXECUTABLE(pcl_pyramid_surface_matching "${SUBSYS_NAME}" src/pyramid_surface_matching.cpp)
  target_link_libraries(pcl_pyramid_surface_matching pcl_common pcl_io pcl_features pcl_registration pcl_filters)

  PCL_ADD_EXECUTABLE(pcl_statistical_multiscale_interest_region_extraction_example "${SUBSYS_NAME}" src/statistical_multiscale_interest_region_extraction_example.cpp)
  target_link_libraries(pcl_statistical_multiscale_interest_region_extraction_example pcl_common pcl_io pcl_features pcl_filters)

  if(LIBUSB_1_FOUND)
    PCL_ADD_EXECUTABLE(pcl_dinast_grabber "${SUBSYS_NAME}" src/dinast_grabber_example.cpp)
    target_link_libraries(pcl_dinast_grabber pcl_common pcl_visualization pcl_io)
  endif(LIBUSB_1_FOUND)

  if (VTK_FOUND)
    PCL_ADD_EXECUTABLE(pcl_ppf_object_recognition "${SUBSYS_NAME}" src/ppf_object_recognition.cpp)
    target_link_libraries(pcl_ppf_object_recognition pcl_common pcl_io pcl_filters pcl_features pcl_registration pcl_visualization pcl_sample_consensus pcl_segmentation)

    PCL_ADD_EXECUTABLE(pcl_multiscale_feature_persistence_example "${SUBSYS_NAME}" src/multiscale_feature_persistence_example.cpp)
    target_link_libraries(pcl_multiscale_feature_persistence_example pcl_common pcl_io pcl_filters pcl_features pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_surfel_smoothing_test "${SUBSYS_NAME}" src/surfel_smoothing_test.cpp)
    target_link_libraries(pcl_surfel_smoothing_test pcl_common pcl_io pcl_surface pcl_filters pcl_features pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_feature_matching "${SUBSYS_NAME}" src/feature_matching.cpp)
    target_link_libraries(pcl_feature_matching pcl_common pcl_io pcl_registration pcl_keypoints pcl_sample_consensus pcl_visualization pcl_search pcl_features pcl_kdtree pcl_surface pcl_segmentation)

    PCL_ADD_EXECUTABLE(pcl_convolve "${SUBSYS_NAME}" src/convolve.cpp)
    target_link_libraries(pcl_convolve pcl_common pcl_io pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_pcd_organized_multi_plane_segmentation "${SUBSYS_NAME}" src/pcd_organized_multi_plane_segmentation.cpp)
    target_link_libraries(pcl_pcd_organized_multi_plane_segmentation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_features)

    if (QHULL_FOUND)
        PCL_ADD_EXECUTABLE(pcl_pcd_select_object_plane "${SUBSYS_NAME}" src/pcd_select_object_plane.cpp)
        target_link_libraries(pcl_pcd_select_object_plane pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_features pcl_surface)
    endif()

#    PCL_ADD_EXECUTABLE(pcl_convolve "${SUBSYS_NAME}" src/convolve.cpp)
#    target_link_libraries(pcl_convolve pcl_common pcl_io pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_pcd_organized_edge_detection "${SUBSYS_NAME}" src/pcd_organized_edge_detection.cpp)
    target_link_libraries(pcl_pcd_organized_edge_detection pcl_common pcl_io pcl_features pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_face_trainer "${SUBSYS_NAME}" src/face_detection/face_trainer.cpp)
    target_link_libraries(pcl_face_trainer pcl_features pcl_recognition pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_surface pcl_keypoints pcl_ml pcl_search pcl_kdtree vtkRendering vtkIO)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_fs_face_detector "${SUBSYS_NAME}" src/face_detection//filesystem_face_detection.cpp)
    target_link_libraries(pcl_fs_face_detector pcl_features pcl_recognition pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_surface pcl_keypoints pcl_ml pcl_search pcl_kdtree vtkRendering vtkIO)

    PCL_ADD_EXECUTABLE(pcl_stereo_ground_segmentation "${SUBSYS_NAME}" src/stereo_ground_segmentation.cpp)
    target_link_libraries(pcl_stereo_ground_segmentation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_features pcl_stereo)

    if (OPENNI_FOUND AND BUILD_OPENNI)

#      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_grab_frame "${SUBSYS_NAME}" src/openni_grab_frame.cpp)
#      target_link_libraries(pcl_openni_grab_frame pcl_common pcl_io pcl_visualization)

#      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_grab_images "${SUBSYS_NAME}" src/openni_grab_images.cpp)
#      target_link_libraries(pcl_openni_grab_images pcl_common pcl_io pcl_visualization)


      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_fast_mesh "${SUBSYS_NAME}" src/openni_fast_mesh.cpp)
      target_link_libraries(pcl_openni_fast_mesh pcl_common pcl_io pcl_visualization pcl_surface)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_voxel_grid "${SUBSYS_NAME}" src/openni_voxel_grid.cpp)
      target_link_libraries(pcl_openni_voxel_grid pcl_common pcl_io pcl_filters pcl_visualization)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_octree_compression "${SUBSYS_NAME}" src/openni_octree_compression.cpp)
      target_link_libraries(pcl_openni_octree_compression pcl_common pcl_io pcl_filters pcl_visualization pcl_octree)

      if(HAVE_PNG)
        PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_organized_compression "${SUBSYS_NAME}" src/openni_organized_compression.cpp)
        target_link_libraries(pcl_openni_organized_compression pcl_common pcl_io pcl_filters pcl_visualization pcl_octree)
      endif(HAVE_PNG)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_shift_to_depth_conversion "${SUBSYS_NAME}" src/openni_shift_to_depth_conversion.cpp)
      target_link_libraries(pcl_openni_shift_to_depth_conversion pcl_common pcl_visualization)

      PCL_ADD_EXECUTABLE(pcl_openni_mobile_server "${SUBSYS_NAME}" src/openni_mobile_server.cpp)
      target_link_libraries(pcl_openni_mobile_server pcl_common pcl_io pcl_filters pcl_visualization)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_planar_segmentation "${SUBSYS_NAME}" src/openni_planar_segmentation.cpp)
      target_link_libraries(pcl_openni_planar_segmentation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_organized_multi_plane_segmentation "${SUBSYS_NAME}" src/openni_organized_multi_plane_segmentation.cpp)
      target_link_libraries(pcl_openni_organized_multi_plane_segmentation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_features)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_ii_normal_estimation "${SUBSYS_NAME}" src/openni_ii_normal_estimation.cpp)
      target_link_libraries(pcl_openni_ii_normal_estimation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_feature_persistence "${SUBSYS_NAME}" src/openni_feature_persistence.cpp)
      target_link_libraries(pcl_openni_feature_persistence pcl_common pcl_io pcl_filters pcl_visualization pcl_features)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_mls_smoothing "${SUBSYS_NAME}" src/openni_mls_smoothing.cpp)
      target_link_libraries(pcl_openni_mls_smoothing pcl_common pcl_io pcl_surface pcl_visualization)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_change_viewer "${SUBSYS_NAME}" src/openni_change_viewer.cpp)
      target_link_libraries(pcl_openni_change_viewer pcl_common pcl_io pcl_kdtree pcl_octree pcl_visualization pcl_filters)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_uniform_sampling "${SUBSYS_NAME}" src/openni_uniform_sampling.cpp)
      target_link_libraries(pcl_openni_uniform_sampling pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface pcl_keypoints)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_boundary_estimation "${SUBSYS_NAME}" src/openni_boundary_estimation.cpp)
      target_link_libraries(pcl_openni_boundary_estimation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_organized_edge_detection "${SUBSYS_NAME}" src/openni_organized_edge_detection.cpp)
      target_link_libraries(pcl_openni_organized_edge_detection pcl_common pcl_io pcl_features pcl_visualization)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_face_detector "${SUBSYS_NAME}" src/face_detection//openni_face_detection.cpp src/face_detection//openni_frame_source.cpp)
      target_link_libraries(pcl_openni_face_detector pcl_features pcl_recognition pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_surface pcl_keypoints pcl_ml pcl_search pcl_kdtree vtkRendering vtkIO)

      if (QT4_FOUND AND VTK_USE_QVTK)
        # OpenNI Passthrough application demo
        QT4_WRAP_UI(openni_passthrough_ui src/openni_passthrough.ui)
        QT4_WRAP_CPP(openni_passthrough_moc include/pcl/apps/openni_passthrough.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
        PCL_ADD_EXECUTABLE(pcl_openni_passthrough "${SUBSYS_NAME}" ${openni_passthrough_ui} ${openni_passthrough_moc} src/openni_passthrough.cpp)
        target_link_libraries(pcl_openni_passthrough pcl_common pcl_io pcl_filters pcl_visualization QVTK ${QT_LIBRARIES})

        # OpenNI Organized Connected Component application demo
        QT4_WRAP_UI(organized_segmentation_demo_ui src/organized_segmentation_demo.ui)
        QT4_WRAP_CPP(organized_segmentation_demo_moc include/pcl/apps/organized_segmentation_demo.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
        PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_organized_segmentation_demo "${SUBSYS_NAME}" ${organized_segmentation_demo_ui} ${organized_segmentation_demo_moc} src/organized_segmentation_demo.cpp)
        target_link_libraries(pcl_organized_segmentation_demo pcl_common pcl_io pcl_visualization pcl_segmentation pcl_features pcl_surface QVTK ${QT_LIBRARIES})

        # Manual registration demo
        QT4_WRAP_UI(manual_registration_ui src/manual_registration/manual_registration.ui)
        QT4_WRAP_CPP(manual_registration_moc include/pcl/apps/manual_registration.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
        PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_manual_registration "${SUBSYS_NAME}" ${manual_registration_ui} ${manual_registration_moc} src/manual_registration/manual_registration.cpp)
        target_link_libraries(pcl_manual_registration pcl_common pcl_io pcl_visualization pcl_segmentation pcl_features pcl_surface QVTK ${QT_LIBRARIES})

        QT4_WRAP_UI(pcd_video_player_ui src/pcd_video_player/pcd_video_player.ui)
        QT4_WRAP_CPP(pcd_video_player_moc include/pcl/apps/pcd_video_player.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
        PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_pcd_video_player "${SUBSYS_NAME}" ${pcd_video_player_ui} ${pcd_video_player_moc} src/pcd_video_player/pcd_video_player.cpp)
        target_link_libraries(pcl_pcd_video_player pcl_common pcl_io pcl_visualization pcl_segmentation pcl_features pcl_surface QVTK ${QT_LIBRARIES})

        # Database processing (integration) demo
#        QT4_WRAP_UI(db_proc_ui src/db_proc/db_proc.ui)
#        QT4_WRAP_CPP(db_proc_moc include/pcl/apps/db_proc.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
#        PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_db_proc "${SUBSYS_NAME}" ${db_proc_ui} ${db_proc_moc} src/db_proc/db_proc.cpp)
#        target_link_libraries(pcl_db_proc pcl_common pcl_io pcl_visualization pcl_segmentation pcl_features pcl_surface QVTK ${QT_LIBRARIES})

      endif ()

      set(incs
        include/pcl/${SUBSYS_NAME}/render_views_tesselated_sphere.h
        include/pcl/${SUBSYS_NAME}/timer.h)
      set(srcs src/render_views_tesselated_sphere.cpp)

      if (QHULL_FOUND)
        PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_3d_convex_hull "${SUBSYS_NAME}" src/openni_3d_convex_hull.cpp)
        target_link_libraries(pcl_openni_3d_convex_hull pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface)

        PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_3d_concave_hull "${SUBSYS_NAME}" src/openni_3d_concave_hull.cpp)
        target_link_libraries(pcl_openni_3d_concave_hull pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface)

        PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_tracking "${SUBSYS_NAME}" src/openni_tracking.cpp)
        target_link_libraries(pcl_openni_tracking pcl_common pcl_io pcl_surface pcl_visualization pcl_filters pcl_features pcl_segmentation pcl_tracking pcl_search)

        set(incs "include/pcl/${SUBSYS_NAME}/dominant_plane_segmentation.h" ${incs})
        set(impl_incs "include/pcl/${SUBSYS_NAME}/impl/dominant_plane_segmentation.hpp")
        set(srcs src/dominant_plane_segmentation.cpp ${srcs})

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_planar_convex_hull "${SUBSYS_NAME}" src/openni_planar_convex_hull.cpp)
      target_link_libraries(pcl_openni_planar_convex_hull pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_surface)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_ni_linemod "${SUBSYS_NAME}" src/ni_linemod.cpp)
      target_link_libraries(pcl_ni_linemod pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface pcl_search)

      endif() # QHULL_FOUND

      # Install include files
      PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
      PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

      set(LIB_NAME "pcl_${SUBSYS_NAME}")
      PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${impl_incs} ${incs})
      target_link_libraries("${LIB_NAME}" pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_surface pcl_features pcl_sample_consensus pcl_search)

      PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "" "" "" "" "")


      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_ni_agast "${SUBSYS_NAME}" src/ni_agast.cpp)
      target_link_libraries(pcl_ni_agast pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_keypoints pcl_surface pcl_search)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_ni_brisk "${SUBSYS_NAME}" src/ni_brisk.cpp)
      target_link_libraries(pcl_ni_brisk pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_keypoints pcl_surface pcl_search)

      PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_ni_susan "${SUBSYS_NAME}" src/ni_susan.cpp)
      target_link_libraries(pcl_ni_susan pcl_common pcl_visualization pcl_features pcl_keypoints pcl_search)

    endif() # OPENNI_FOUND + BUILD_OPENNI

  endif() # VTK_FOUND

  # OpenGL and GLUT
  if(OPENGL_FOUND AND GLUT_FOUND)
    include_directories("${OPENGL_INCLUDE_DIR}")
    include_directories("${GLUT_INCLUDE_DIR}")
    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_grabcut_2d "${SUBSYS_NAME}" src/grabcut_2d.cpp)
    target_link_libraries (pcl_grabcut_2d pcl_common pcl_io pcl_segmentation pcl_search ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})
  endif(OPENGL_FOUND AND GLUT_FOUND)

  collect_subproject_directory_names("${CMAKE_CURRENT_SOURCE_DIR}" "CMakeLists.txt" PCL_APPS_MODULES_NAMES PCL_APPS_MODULES_DIRS ${SUBSYS_NAME})
  set(PCL_APPS_MODULES_NAMES_UNSORTED ${PCL_APPS_MODULES_NAMES})
  topological_sort(PCL_APPS_MODULES_NAMES PCL_APPS_ _DEPENDS)
  sort_relative(PCL_APPS_MODULES_NAMES_UNSORTED PCL_APPS_MODULES_NAMES PCL_APPS_MODULES_DIRS)
  foreach(subdir ${PCL_APPS_MODULES_DIRS})
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/${subdir}")
  endforeach(subdir)

endif(build)
