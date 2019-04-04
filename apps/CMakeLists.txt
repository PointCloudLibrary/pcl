set(SUBSYS_NAME apps)
set(SUBSYS_DESC "Application examples/samples that show how PCL works")
set(SUBSYS_DEPS common geometry io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search recognition ml stereo 2d)

set(DEFAULT FALSE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} OPT_DEPS openni vtk)

if(NOT build)
  return()
endif()

# to be filled with all targets the apps subsystem
set(PCL_APPS_ALL_TARGETS)


PCL_ADD_EXECUTABLE(pcl_test_search_speed "${SUBSYS_NAME}" src/test_search.cpp)

PCL_ADD_EXECUTABLE(pcl_nn_classification_example "${SUBSYS_NAME}" src/nn_classification_example.cpp)

PCL_ADD_EXECUTABLE(pcl_pyramid_surface_matching "${SUBSYS_NAME}" src/pyramid_surface_matching.cpp)

PCL_ADD_EXECUTABLE(pcl_statistical_multiscale_interest_region_extraction_example "${SUBSYS_NAME}" src/statistical_multiscale_interest_region_extraction_example.cpp)

if(LIBUSB_1_FOUND)
  PCL_ADD_EXECUTABLE(pcl_dinast_grabber "${SUBSYS_NAME}" src/dinast_grabber_example.cpp)
endif()

if(VTK_FOUND)
  include("${VTK_USE_FILE}")

  set(incs "include/pcl/${SUBSYS_NAME}/render_views_tesselated_sphere.h")
  set(srcs "src/render_views_tesselated_sphere.cpp")

  PCL_ADD_EXECUTABLE(pcl_ppf_object_recognition "${SUBSYS_NAME}" src/ppf_object_recognition.cpp)

  PCL_ADD_EXECUTABLE(pcl_multiscale_feature_persistence_example "${SUBSYS_NAME}" src/multiscale_feature_persistence_example.cpp)

  PCL_ADD_EXECUTABLE(pcl_surfel_smoothing_test "${SUBSYS_NAME}" src/surfel_smoothing_test.cpp)

  PCL_ADD_EXECUTABLE(pcl_feature_matching "${SUBSYS_NAME}" src/feature_matching.cpp)

  PCL_ADD_EXECUTABLE(pcl_convolve "${SUBSYS_NAME}" src/convolve.cpp)

  PCL_ADD_EXECUTABLE(pcl_pcd_organized_multi_plane_segmentation "${SUBSYS_NAME}" src/pcd_organized_multi_plane_segmentation.cpp)

  if(QHULL_FOUND)
    PCL_ADD_EXECUTABLE(pcl_pcd_select_object_plane "${SUBSYS_NAME}" src/pcd_select_object_plane.cpp)
  endif()

  PCL_ADD_EXECUTABLE(pcl_pcd_organized_edge_detection "${SUBSYS_NAME}" src/pcd_organized_edge_detection.cpp)

  PCL_ADD_EXECUTABLE(pcl_face_trainer "${SUBSYS_NAME}" src/face_detection/face_trainer.cpp)

  PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_fs_face_detector "${SUBSYS_NAME}" src/face_detection//filesystem_face_detection.cpp)

  PCL_ADD_EXECUTABLE(pcl_stereo_ground_segmentation "${SUBSYS_NAME}" src/stereo_ground_segmentation.cpp)

  if(Qt5_FOUND AND VTK_USE_QVTK)
    # Manual registration demo
    QT5_WRAP_UI(manual_registration_ui src/manual_registration/manual_registration.ui)
    QT5_WRAP_CPP(manual_registration_moc include/pcl/apps/manual_registration.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_manual_registration "${SUBSYS_NAME}" ${manual_registration_ui} ${manual_registration_moc} src/manual_registration/manual_registration.cpp)

    QT5_WRAP_UI(pcd_video_player_ui src/pcd_video_player/pcd_video_player.ui)
    QT5_WRAP_CPP(pcd_video_player_moc include/pcl/apps/pcd_video_player.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_pcd_video_player "${SUBSYS_NAME}" ${pcd_video_player_ui} ${pcd_video_player_moc} src/pcd_video_player/pcd_video_player.cpp)
  endif()

  if(WITH_OPENNI)
    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_fast_mesh "${SUBSYS_NAME}" src/openni_fast_mesh.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_voxel_grid "${SUBSYS_NAME}" src/openni_voxel_grid.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_octree_compression "${SUBSYS_NAME}" src/openni_octree_compression.cpp)

    if(HAVE_PNG)
    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_organized_compression "${SUBSYS_NAME}" src/openni_organized_compression.cpp)
    endif()

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_shift_to_depth_conversion "${SUBSYS_NAME}" src/openni_shift_to_depth_conversion.cpp)

    PCL_ADD_EXECUTABLE(pcl_openni_mobile_server "${SUBSYS_NAME}" src/openni_mobile_server.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_planar_segmentation "${SUBSYS_NAME}" src/openni_planar_segmentation.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_organized_multi_plane_segmentation "${SUBSYS_NAME}" src/openni_organized_multi_plane_segmentation.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_ii_normal_estimation "${SUBSYS_NAME}" src/openni_ii_normal_estimation.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_feature_persistence "${SUBSYS_NAME}" src/openni_feature_persistence.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_mls_smoothing "${SUBSYS_NAME}" src/openni_mls_smoothing.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_change_viewer "${SUBSYS_NAME}" src/openni_change_viewer.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_uniform_sampling "${SUBSYS_NAME}" src/openni_uniform_sampling.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_boundary_estimation "${SUBSYS_NAME}" src/openni_boundary_estimation.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_organized_edge_detection "${SUBSYS_NAME}" src/openni_organized_edge_detection.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_face_detector "${SUBSYS_NAME}" src/face_detection//openni_face_detection.cpp src/face_detection//openni_frame_source.cpp)

    if(Qt5_FOUND AND VTK_USE_QVTK)
    # OpenNI Passthrough application demo
    QT5_WRAP_UI(openni_passthrough_ui src/openni_passthrough.ui)
    QT5_WRAP_CPP(openni_passthrough_moc include/pcl/apps/openni_passthrough.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
    PCL_ADD_EXECUTABLE(pcl_openni_passthrough "${SUBSYS_NAME}" ${openni_passthrough_ui} ${openni_passthrough_moc} src/openni_passthrough.cpp)

    # OpenNI Organized Connected Component application demo
    QT5_WRAP_UI(organized_segmentation_demo_ui src/organized_segmentation_demo.ui)
    QT5_WRAP_CPP(organized_segmentation_demo_moc include/pcl/apps/organized_segmentation_demo.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_organized_segmentation_demo "${SUBSYS_NAME}" ${organized_segmentation_demo_ui} ${organized_segmentation_demo_moc} src/organized_segmentation_demo.cpp)
    endif()

    if(QHULL_FOUND)
    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_3d_convex_hull "${SUBSYS_NAME}" src/openni_3d_convex_hull.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_3d_concave_hull "${SUBSYS_NAME}" src/openni_3d_concave_hull.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_tracking "${SUBSYS_NAME}" src/openni_tracking.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_planar_convex_hull "${SUBSYS_NAME}" src/openni_planar_convex_hull.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_ni_linemod "${SUBSYS_NAME}" src/ni_linemod.cpp)
    endif() # QHULL_FOUND

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_ni_agast "${SUBSYS_NAME}" src/ni_agast.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_ni_brisk "${SUBSYS_NAME}" src/ni_brisk.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_ni_susan "${SUBSYS_NAME}" src/ni_susan.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_ni_trajkovic ${SUBSYS_NAME} src/ni_trajkovic.cpp)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_openni_klt "${SUBSYS_NAME}" src/openni_klt.cpp)
  endif() # WITH_OPENNI
endif() # VTK_FOUND

# OpenGL and GLUT
if(OPENGL_FOUND AND GLUT_FOUND)
  if(NOT WIN32)
    include_directories(SYSTEM "${OPENGL_INCLUDE_DIR}")
  endif()
  PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_grabcut_2d "${SUBSYS_NAME}" src/grabcut_2d.cpp)
  target_link_libraries (pcl_grabcut_2d pcl_common pcl_io pcl_segmentation pcl_search GLUT::GLUT ${OPENGL_LIBRARIES})
endif()

collect_subproject_directory_names("${CMAKE_CURRENT_SOURCE_DIR}" "CMakeLists.txt" PCL_APPS_MODULES_NAMES PCL_APPS_MODULES_DIRS ${SUBSYS_NAME})
set(PCL_APPS_MODULES_NAMES_UNSORTED ${PCL_APPS_MODULES_NAMES})
topological_sort(PCL_APPS_MODULES_NAMES PCL_APPS_ _DEPENDS)
sort_relative(PCL_APPS_MODULES_NAMES_UNSORTED PCL_APPS_MODULES_NAMES PCL_APPS_MODULES_DIRS)
foreach(subdir ${PCL_APPS_MODULES_DIRS})
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/${subdir}")
endforeach()

if(QHULL_FOUND)
  set(incs
    "include/pcl/${SUBSYS_NAME}/dominant_plane_segmentation.h"
    "include/pcl/${SUBSYS_NAME}/timer.h"
    ${incs}
  )
  set(impl_incs "include/pcl/${SUBSYS_NAME}/impl/dominant_plane_segmentation.hpp")
  set(srcs "src/dominant_plane_segmentation.cpp" ${srcs})
endif()

set(LIB_NAME "pcl_${SUBSYS_NAME}")
PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${impl_incs} ${incs})
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC})
# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

add_custom_target(${SUBSYS_NAME} DEPENDS ${PCL_APPS_ALL_TARGETS})