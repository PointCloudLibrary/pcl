set(SUBSYS_NAME global_tests)
set(SUBSYS_DESC "Point cloud library global tests")
set(SUBSYS_DEPS common sample_consensus io kdtree features filters keypoints search surface registration segmentation octree)

find_package(GTest)
if(GTEST_FOUND)
    set(REASON)
    set(DEFAULT ON)
else(GTEST_FOUND)
    set(REASON "GTest was not found.")
    set(DEFAULT OFF)
endif(GTEST_FOUND)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC(${SUBSYS_NAME})

if(build)
    enable_testing()
    include_directories(${GTEST_INCLUDE_DIRS})
    include_directories(${PCL_INCLUDE_DIRS})

    if(QHULL_FOUND)
      PCL_ADD_TEST(a_surface_test test_surface
                   FILES test_surface.cpp
                   LINK_WITH pcl_io pcl_kdtree pcl_surface pcl_features
                   ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
    endif(QHULL_FOUND)
    
    PCL_ADD_TEST(search test_search
                 FILES test_search.cpp
                 LINK_WITH pcl_search pcl_io pcl_kdtree
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/table_scene_mug_stereo_textured.pcd)

    PCL_ADD_TEST(filters_filters test_filters
                 FILES test_filters.cpp
                 LINK_WITH pcl_filters pcl_io pcl_sample_consensus pcl_kdtree
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
    
    PCL_ADD_TEST(a_sample_consensus_test test_sample_consensus
                 FILES test_sample_consensus.cpp
                 LINK_WITH pcl_io pcl_sample_consensus pcl_kdtree pcl_features
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/sac_plane_test.pcd)
    
    PCL_ADD_TEST(a_transforms_test test_transforms
                 FILES test_transforms.cpp
                 LINK_WITH pcl_io
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
    
    PCL_ADD_TEST(a_segmentation_test test_segmentation
                 FILES test_segmentation.cpp
                 LINK_WITH pcl_io pcl_segmentation
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd)
    
    PCL_ADD_TEST(a_registration_test test_registration
                 FILES test_registration.cpp
                 LINK_WITH pcl_io pcl_registration pcl_features pcl_search pcl_kdtree
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd ${PCL_SOURCE_DIR}/test/bun4.pcd)
    
    PCL_ADD_TEST(registration_api test_registration_api
                 FILES test_registration_api.cpp test_registration_api_data.h
                 LINK_WITH pcl_io pcl_registration pcl_features
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/bun0.pcd ${PCL_SOURCE_DIR}/test/bun4.pcd)
    
    PCL_ADD_TEST(a_keypoints_test test_keypoints
                 FILES test_keypoints.cpp
                 LINK_WITH pcl_io pcl_kdtree pcl_filters pcl_keypoints
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/cturtle.pcd)
    
    PCL_ADD_TEST(a_ii_normals_test test_ii_normals
                 FILES test_ii_normals.cpp
                 LINK_WITH pcl_io pcl_features
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/table_scene_mug_stereo_textured.pcd)

    PCL_ADD_TEST(test_non_linear test_non_linear
                 FILES test_non_linear.cpp
                 LINK_WITH pcl_common pcl_io pcl_sample_consensus pcl_segmentation
                 ARGUMENTS ${PCL_SOURCE_DIR}/test/noisy_slice_displaced.pcd)

                #Needs office1.pcd
               #PCL_ADD_TEST(organized_index test_organized_index
               #  FILES test_organized_index.cpp
               #  LINK_WITH pcl_search pcl_io pcl_filters)
    add_subdirectory(features)
  
endif(build)
