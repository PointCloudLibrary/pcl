set(SUBSYS_NAME apps)
set(SUBSYS_DESC "Application examples/samples that show how PCL works")
set(SUBSYS_DEPS common geometry io filters sample_consensus segmentation visualization kdtree features surface octree registration keypoints tracking search recognition ml stereo 2d)
set(SUBSYS_OPT_DEPS openni vtk ${QTX})

set(DEFAULT FALSE)
set(REASON "Disabled by default")
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${SUBSYS_OPT_DEPS})

if(NOT build)
  return()
endif()

#Enable cmakes QT auto features.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# to be filled with all targets the apps subsystem
set(PCL_APPS_ALL_TARGETS)

include_directories("${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/include")

PCL_ADD_EXECUTABLE(pcl_test_search_speed COMPONENT ${SUBSYS_NAME} SOURCES src/test_search.cpp)
target_link_libraries(pcl_test_search_speed pcl_common pcl_io pcl_search pcl_kdtree pcl_visualization)

PCL_ADD_EXECUTABLE(pcl_nn_classification_example COMPONENT ${SUBSYS_NAME} SOURCES src/nn_classification_example.cpp)
target_link_libraries(pcl_nn_classification_example pcl_common pcl_io pcl_features pcl_kdtree)

PCL_ADD_EXECUTABLE(pcl_pyramid_surface_matching COMPONENT ${SUBSYS_NAME} SOURCES src/pyramid_surface_matching.cpp)
target_link_libraries(pcl_pyramid_surface_matching pcl_common pcl_io pcl_features pcl_registration pcl_filters)

PCL_ADD_EXECUTABLE(pcl_statistical_multiscale_interest_region_extraction_example COMPONENT ${SUBSYS_NAME} SOURCES src/statistical_multiscale_interest_region_extraction_example.cpp)
target_link_libraries(pcl_statistical_multiscale_interest_region_extraction_example pcl_common pcl_io pcl_features pcl_filters)

if(LIBUSB_FOUND)
  PCL_ADD_EXECUTABLE(pcl_dinast_grabber COMPONENT ${SUBSYS_NAME} SOURCES src/dinast_grabber_example.cpp)
  target_link_libraries(pcl_dinast_grabber pcl_common pcl_visualization pcl_io)
endif()

if(VTK_FOUND)
  set(incs "include/pcl/${SUBSYS_NAME}/render_views_tesselated_sphere.h")
  set(srcs "src/render_views_tesselated_sphere.cpp")

  PCL_ADD_EXECUTABLE(pcl_ppf_object_recognition COMPONENT ${SUBSYS_NAME} SOURCES src/ppf_object_recognition.cpp)
  target_link_libraries(pcl_ppf_object_recognition pcl_common pcl_io pcl_filters pcl_features pcl_registration pcl_visualization pcl_sample_consensus pcl_segmentation)

  PCL_ADD_EXECUTABLE(pcl_multiscale_feature_persistence_example COMPONENT ${SUBSYS_NAME} SOURCES src/multiscale_feature_persistence_example.cpp)
  target_link_libraries(pcl_multiscale_feature_persistence_example pcl_common pcl_io pcl_filters pcl_features pcl_visualization)

  PCL_ADD_EXECUTABLE(pcl_surfel_smoothing_test COMPONENT ${SUBSYS_NAME} SOURCES src/surfel_smoothing_test.cpp)
  target_link_libraries(pcl_surfel_smoothing_test pcl_common pcl_io pcl_surface pcl_filters pcl_features pcl_visualization)

  PCL_ADD_EXECUTABLE(pcl_feature_matching COMPONENT ${SUBSYS_NAME} SOURCES src/feature_matching.cpp)
  target_link_libraries(pcl_feature_matching pcl_common pcl_io pcl_registration pcl_keypoints pcl_sample_consensus pcl_visualization pcl_search pcl_features pcl_kdtree pcl_surface pcl_segmentation)

  PCL_ADD_EXECUTABLE(pcl_convolve COMPONENT ${SUBSYS_NAME} SOURCES src/convolve.cpp)
  target_link_libraries(pcl_convolve pcl_common pcl_filters pcl_io pcl_visualization)

  PCL_ADD_EXECUTABLE(pcl_pcd_organized_multi_plane_segmentation COMPONENT ${SUBSYS_NAME} SOURCES src/pcd_organized_multi_plane_segmentation.cpp)
  target_link_libraries(pcl_pcd_organized_multi_plane_segmentation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_features)

  if(QHULL_FOUND)
    PCL_ADD_EXECUTABLE(pcl_pcd_select_object_plane COMPONENT ${SUBSYS_NAME} SOURCES src/pcd_select_object_plane.cpp)
    target_link_libraries(pcl_pcd_select_object_plane pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_features pcl_surface)
  endif()

  PCL_ADD_EXECUTABLE(pcl_pcd_organized_edge_detection COMPONENT ${SUBSYS_NAME} SOURCES src/pcd_organized_edge_detection.cpp)
  target_link_libraries(pcl_pcd_organized_edge_detection pcl_common pcl_io pcl_features pcl_visualization)

  PCL_ADD_EXECUTABLE(pcl_face_trainer COMPONENT ${SUBSYS_NAME} SOURCES src/face_detection/face_trainer.cpp)
  target_link_libraries(pcl_face_trainer pcl_features pcl_recognition pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_surface pcl_keypoints pcl_ml pcl_search pcl_kdtree)

  PCL_ADD_EXECUTABLE(pcl_fs_face_detector COMPONENT ${SUBSYS_NAME} SOURCES src/face_detection//filesystem_face_detection.cpp BUNDLE)
  target_link_libraries(pcl_fs_face_detector pcl_features pcl_recognition pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_surface pcl_keypoints pcl_ml pcl_search pcl_kdtree)

  PCL_ADD_EXECUTABLE(pcl_stereo_ground_segmentation COMPONENT ${SUBSYS_NAME} SOURCES src/stereo_ground_segmentation.cpp)
  target_link_libraries(pcl_stereo_ground_segmentation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_features pcl_stereo)

  if(QT_FOUND AND HAVE_QVTK)
    # Manual registration demo
    PCL_ADD_EXECUTABLE(pcl_manual_registration
      COMPONENT
      ${SUBSYS_NAME}
      SOURCES
      include/pcl/apps/manual_registration.h
      src/manual_registration/manual_registration.cpp
      src/manual_registration/manual_registration.ui
      BUNDLE)
      
    target_link_libraries(pcl_manual_registration pcl_common pcl_io pcl_visualization pcl_segmentation pcl_features pcl_surface ${QTX}::Widgets)

    PCL_ADD_EXECUTABLE(pcl_pcd_video_player
      COMPONENT
      ${SUBSYS_NAME}
      SOURCES
      include/pcl/apps/pcd_video_player.h
      src/pcd_video_player/pcd_video_player.cpp
      src/pcd_video_player/pcd_video_player.ui
      BUNDLE)
    
    target_link_libraries(pcl_pcd_video_player pcl_common pcl_io pcl_visualization pcl_segmentation pcl_features pcl_surface ${QTX}::Widgets)
  endif()

  if(WITH_OPENNI)
    PCL_ADD_EXECUTABLE(pcl_openni_fast_mesh COMPONENT ${SUBSYS_NAME} SOURCES src/openni_fast_mesh.cpp BUNDLE)
    target_link_libraries(pcl_openni_fast_mesh pcl_common pcl_io pcl_visualization pcl_surface)

    PCL_ADD_EXECUTABLE(pcl_openni_voxel_grid COMPONENT ${SUBSYS_NAME} SOURCES src/openni_voxel_grid.cpp BUNDLE)
    target_link_libraries(pcl_openni_voxel_grid pcl_common pcl_io pcl_filters pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_openni_octree_compression COMPONENT ${SUBSYS_NAME} SOURCES src/openni_octree_compression.cpp BUNDLE)
    target_link_libraries(pcl_openni_octree_compression pcl_common pcl_io pcl_filters pcl_visualization pcl_octree)

    if(HAVE_PNG)
    PCL_ADD_EXECUTABLE(pcl_openni_organized_compression COMPONENT ${SUBSYS_NAME} SOURCES src/openni_organized_compression.cpp BUNDLE)
    target_link_libraries(pcl_openni_organized_compression pcl_common pcl_io pcl_filters pcl_visualization pcl_octree)
    endif()

    PCL_ADD_EXECUTABLE(pcl_openni_shift_to_depth_conversion COMPONENT ${SUBSYS_NAME} SOURCES src/openni_shift_to_depth_conversion.cpp BUNDLE)
    target_link_libraries(pcl_openni_shift_to_depth_conversion pcl_common pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_openni_mobile_server COMPONENT ${SUBSYS_NAME} SOURCES src/openni_mobile_server.cpp)
    target_link_libraries(pcl_openni_mobile_server pcl_common pcl_io pcl_filters pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_openni_planar_segmentation COMPONENT ${SUBSYS_NAME} SOURCES src/openni_planar_segmentation.cpp BUNDLE)
    target_link_libraries(pcl_openni_planar_segmentation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus)

    PCL_ADD_EXECUTABLE(pcl_openni_organized_multi_plane_segmentation COMPONENT ${SUBSYS_NAME} SOURCES src/openni_organized_multi_plane_segmentation.cpp BUNDLE)
    target_link_libraries(pcl_openni_organized_multi_plane_segmentation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_features)

    PCL_ADD_EXECUTABLE(pcl_openni_ii_normal_estimation COMPONENT ${SUBSYS_NAME} SOURCES src/openni_ii_normal_estimation.cpp BUNDLE)
    target_link_libraries(pcl_openni_ii_normal_estimation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface)

    PCL_ADD_EXECUTABLE(pcl_openni_feature_persistence COMPONENT ${SUBSYS_NAME} SOURCES src/openni_feature_persistence.cpp BUNDLE)
    target_link_libraries(pcl_openni_feature_persistence pcl_common pcl_io pcl_filters pcl_visualization pcl_features)

    PCL_ADD_EXECUTABLE(pcl_openni_mls_smoothing COMPONENT ${SUBSYS_NAME} SOURCES src/openni_mls_smoothing.cpp BUNDLE)
    target_link_libraries(pcl_openni_mls_smoothing pcl_common pcl_io pcl_surface pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_openni_change_viewer COMPONENT ${SUBSYS_NAME} SOURCES src/openni_change_viewer.cpp BUNDLE)
    target_link_libraries(pcl_openni_change_viewer pcl_common pcl_io pcl_kdtree pcl_octree pcl_visualization pcl_filters)

    PCL_ADD_EXECUTABLE(pcl_openni_uniform_sampling COMPONENT ${SUBSYS_NAME} SOURCES src/openni_uniform_sampling.cpp BUNDLE)
    target_link_libraries(pcl_openni_uniform_sampling pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface pcl_keypoints)

    PCL_ADD_EXECUTABLE(pcl_openni_boundary_estimation COMPONENT ${SUBSYS_NAME} SOURCES src/openni_boundary_estimation.cpp BUNDLE)
    target_link_libraries(pcl_openni_boundary_estimation pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface)

    PCL_ADD_EXECUTABLE(pcl_openni_organized_edge_detection COMPONENT ${SUBSYS_NAME} SOURCES src/openni_organized_edge_detection.cpp BUNDLE)
    target_link_libraries(pcl_openni_organized_edge_detection pcl_common pcl_io pcl_features pcl_visualization)

    PCL_ADD_EXECUTABLE(pcl_openni_face_detector COMPONENT ${SUBSYS_NAME} SOURCES src/face_detection//openni_face_detection.cpp src/face_detection//openni_frame_source.cpp BUNDLE)
    target_link_libraries(pcl_openni_face_detector pcl_features pcl_recognition pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_surface pcl_keypoints pcl_ml pcl_search pcl_kdtree)

    if(QT_FOUND AND HAVE_QVTK)
      # OpenNI Passthrough application demo
      PCL_ADD_EXECUTABLE(pcl_openni_passthrough
        COMPONENT
          ${SUBSYS_NAME}
        SOURCES
          include/pcl/apps/openni_passthrough.h
          src/openni_passthrough.cpp
          src/openni_passthrough.ui)
          
      target_link_libraries(pcl_openni_passthrough pcl_common pcl_io pcl_filters pcl_visualization ${QTX}::Widgets)

      list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS "src")

      # OpenNI Organized Connected Component application demo
      PCL_ADD_EXECUTABLE(pcl_organized_segmentation_demo
        COMPONENT
          ${SUBSYS_NAME}
        SOURCES
          include/pcl/apps/organized_segmentation_demo_qt.h
          include/pcl/apps/organized_segmentation_demo.h
          src/organized_segmentation_demo.cpp
          src/organized_segmentation_demo.ui
        BUNDLE)
      target_link_libraries(pcl_organized_segmentation_demo pcl_common pcl_io pcl_visualization pcl_segmentation pcl_features pcl_surface ${QTX}::Widgets)

    endif()

    if(QHULL_FOUND)
    PCL_ADD_EXECUTABLE(pcl_openni_3d_convex_hull COMPONENT ${SUBSYS_NAME} SOURCES src/openni_3d_convex_hull.cpp BUNDLE)
    target_link_libraries(pcl_openni_3d_convex_hull pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface)

    PCL_ADD_EXECUTABLE(pcl_openni_3d_concave_hull COMPONENT ${SUBSYS_NAME} SOURCES src/openni_3d_concave_hull.cpp BUNDLE)
    target_link_libraries(pcl_openni_3d_concave_hull pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface)

    PCL_ADD_EXECUTABLE(pcl_openni_tracking COMPONENT ${SUBSYS_NAME} SOURCES src/openni_tracking.cpp BUNDLE)
    target_link_libraries(pcl_openni_tracking pcl_common pcl_io pcl_surface pcl_visualization pcl_filters pcl_features pcl_segmentation pcl_tracking pcl_search)

    PCL_ADD_EXECUTABLE(pcl_openni_planar_convex_hull COMPONENT ${SUBSYS_NAME} SOURCES src/openni_planar_convex_hull.cpp BUNDLE)
    target_link_libraries(pcl_openni_planar_convex_hull pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_surface)

    PCL_ADD_EXECUTABLE(pcl_ni_linemod COMPONENT ${SUBSYS_NAME} SOURCES src/ni_linemod.cpp BUNDLE)
    target_link_libraries(pcl_ni_linemod pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_surface pcl_search)
    endif() # QHULL_FOUND

    PCL_ADD_EXECUTABLE(pcl_ni_agast COMPONENT ${SUBSYS_NAME} SOURCES src/ni_agast.cpp BUNDLE)
    target_link_libraries(pcl_ni_agast pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_keypoints pcl_surface pcl_search)

    PCL_ADD_EXECUTABLE(pcl_ni_brisk COMPONENT ${SUBSYS_NAME} SOURCES src/ni_brisk.cpp BUNDLE)
    target_link_libraries(pcl_ni_brisk pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_sample_consensus pcl_features pcl_keypoints pcl_surface pcl_search)

    PCL_ADD_EXECUTABLE(pcl_ni_susan COMPONENT ${SUBSYS_NAME} SOURCES src/ni_susan.cpp BUNDLE)
    target_link_libraries(pcl_ni_susan pcl_common pcl_visualization pcl_features pcl_keypoints pcl_search)

    PCL_ADD_EXECUTABLE(pcl_ni_trajkovic COMPONENT ${SUBSYS_NAME} SOURCES src/ni_trajkovic.cpp BUNDLE)
    target_link_libraries(pcl_ni_trajkovic pcl_common pcl_visualization pcl_features pcl_keypoints pcl_search)

    PCL_ADD_EXECUTABLE(pcl_openni_klt COMPONENT ${SUBSYS_NAME} SOURCES src/openni_klt.cpp BUNDLE)
    target_link_libraries(pcl_openni_klt pcl_common pcl_io pcl_visualization pcl_tracking)
  endif() # WITH_OPENNI
endif() # VTK_FOUND

# OpenGL and GLUT
if(OPENGL_FOUND AND GLUT_FOUND)
  if(NOT WIN32)
    include_directories(SYSTEM "${OPENGL_INCLUDE_DIR}")
  endif()
  PCL_ADD_EXECUTABLE(pcl_grabcut_2d COMPONENT ${SUBSYS_NAME} SOURCES src/grabcut_2d.cpp BUNDLE)
  if(APPLE)
    set(_glut_target ${GLUT_glut_LIBRARY})
  else()
    set(_glut_target GLUT::GLUT)
  endif()
  target_link_libraries (pcl_grabcut_2d pcl_common pcl_io pcl_segmentation pcl_search ${_glut_target} ${OPENGL_LIBRARIES})
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
PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${impl_incs} ${incs})
target_link_libraries("${LIB_NAME}" pcl_common pcl_io pcl_filters pcl_visualization pcl_segmentation pcl_surface pcl_features pcl_sample_consensus pcl_search)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC})
# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

if(CMAKE_GENERATOR_IS_IDE)
    set(SUBSYS_TARGET_NAME APPS_BUILD)
else()
    set(SUBSYS_TARGET_NAME ${SUBSYS_NAME})
endif()
add_custom_target(${SUBSYS_TARGET_NAME} DEPENDS ${PCL_APPS_ALL_TARGETS})
set_target_properties(${SUBSYS_TARGET_NAME} PROPERTIES FOLDER "Apps")
