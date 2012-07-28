set(SUBSYS_NAME global_tests)
set(SUBSYS_DESC "Point cloud library global tests")

if(BUILD_visualization)
  include (${VTK_USE_FILE})
  set(SUBSYS_DEPS 2d common sample_consensus io kdtree features filters geometry keypoints search surface registration segmentation octree recognition outofcore visualization)
  set(OPT_DEPS vtk)
else()
  set(SUBSYS_DEPS 2d common sample_consensus io kdtree features filters geometry keypoints search surface registration segmentation octree recognition outofcore)
endif()

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

PCL_ADD_DOC(${SUBSYS_NAME})

if(build)

    include_directories(${PCL_SOURCE_DIR}/test/gtest-1.6.0/include)
    include_directories(${PCL_SOURCE_DIR}/test/gtest-1.6.0/)
    add_library(pcl_gtest STATIC gtest-1.6.0/src/gtest-all.cc)

    enable_testing()
    include_directories(${PCL_INCLUDE_DIRS})

    add_subdirectory(2d)
    add_subdirectory(common)
    add_subdirectory(features)
    add_subdirectory(filters)
    add_subdirectory(geometry)
    add_subdirectory(io)
    add_subdirectory(kdtree)
    add_subdirectory(keypoints)
    add_subdirectory(octree)
    add_subdirectory(outofcore)
    add_subdirectory(registration)
    add_subdirectory(search)
    
    if(QHULL_FOUND)
      PCL_ADD_TEST(a_surface_test test_surface
                   FILES test_surface.cpp
                   LINK_WITH pcl_gtest pcl_io pcl_kdtree pcl_surface pcl_features
                   ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
    endif(QHULL_FOUND)
    
    PCL_ADD_TEST(search test_search
                 FILES test_search.cpp
                 LINK_WITH pcl_gtest pcl_search pcl_io pcl_kdtree
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/table_scene_mug_stereo_textured.pcd)

    PCL_ADD_TEST(a_sample_consensus_test test_sample_consensus
                 FILES test_sample_consensus.cpp
                 LINK_WITH pcl_gtest pcl_io pcl_sample_consensus pcl_kdtree pcl_features
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/sac_plane_test.pcd)
    
    PCL_ADD_TEST(a_transforms_test test_transforms
                 FILES test_transforms.cpp
                 LINK_WITH pcl_gtest pcl_io
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
    
    PCL_ADD_TEST(a_segmentation_test test_segmentation
                 FILES test_segmentation.cpp
                 LINK_WITH pcl_gtest pcl_io pcl_segmentation pcl_features
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd ${PCL_SOURCE_DIR}/test/car6.pcd ${PCL_SOURCE_DIR}/test/colored_cloud.pcd)
    
    PCL_ADD_TEST(a_keypoints_test test_keypoints
                 FILES test_keypoints.cpp
                 LINK_WITH pcl_gtest pcl_io pcl_kdtree pcl_filters pcl_keypoints
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/cturtle.pcd)
    
    PCL_ADD_TEST(test_non_linear test_non_linear
                 FILES test_non_linear.cpp
                 LINK_WITH pcl_gtest pcl_common pcl_io pcl_sample_consensus pcl_segmentation
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/noisy_slice_displaced.pcd)

    PCL_ADD_TEST(a_recognition_cg_test test_recognition_cg
                 FILES test_recognition_cg.cpp
                 LINK_WITH pcl_gtest pcl_common pcl_io pcl_kdtree pcl_features pcl_recognition pcl_keypoints
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/milk.pcd ${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd)
endif(build)
