set(SUBSYS_NAME global_tests)
set(SUBSYS_DESC "Point cloud library global tests")
set(SUBSYS_DEPS common sample_consensus io kdtree features range_image filters
    keypoints surface registration segmentation octree)

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
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    enable_testing()
    include_directories(${GTEST_INCLUDE_DIRS})

    add_executable (test_feature test_feature.cpp)
    PCL_ADD_OPENMP_FLAGS(test_feature)
    target_link_libraries (test_feature pcl_features pcl_kdtree pcl_io
        ${GTEST_BOTH_LIBRARIES} gomp)
    PCL_LINK_OPENMP(test_feature)

    add_executable (test_sample_consensus test_sample_consensus.cpp)
    PCL_ADD_OPENMP_FLAGS(test_sample_consensus)
    target_link_libraries (test_sample_consensus pcl_io pcl_sample_consensus pcl_kdtree pcl_features ${GTEST_BOTH_LIBRARIES})
    PCL_LINK_OPENMP(test_sample_consensus)

    add_executable (test_transforms test_transforms.cpp)
    target_link_libraries (test_transforms pcl_io ${GTEST_BOTH_LIBRARIES})

    add_executable (test_segmentation test_segmentation.cpp)
    target_link_libraries (test_segmentation pcl_io pcl_segmentation ${GTEST_BOTH_LIBRARIES})

    add_executable (test_registration test_registration.cpp)
    PCL_ADD_OPENMP_FLAGS(test_registration)
    target_link_libraries (test_registration pcl_io pcl_registration pcl_features ${GTEST_BOTH_LIBRARIES})
    target_link_libraries (test_registration ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
    PCL_LINK_OPENMP(test_registration)

    add_executable (test_keypoints test_keypoints.cpp)
    target_link_libraries (test_keypoints pcl_io pcl_kdtree pcl_filters ${GTEST_BOTH_LIBRARIES})

    add_executable (test_ii_normals test_ii_normals.cpp)
    PCL_ADD_OPENMP_FLAGS(test_ii_normals)
    target_link_libraries (test_ii_normals pcl_io pcl_features ${GTEST_BOTH_LIBRARIES})
    PCL_LINK_OPENMP(test_ii_normals)

    add_executable (test_octree test_octree.cpp)
    target_link_libraries (test_octree pcl_octree ${GTEST_BOTH_LIBRARIES})
    PCL_ADD_OPENMP_FLAGS(test_octree)

    add_test(a_keypoints_test test_keypoints)
    add_test(a_sample_consensus_test test_sample_consensus)
    add_test(a_transforms_test test_transforms)
    add_test(a_registration_test test_registration)
    add_test(a_feature_test test_feature ${PCL_SOURCE_DIR}/test/bun0.pcd)
    add_test(a_ii_normals_test test_ii_normals)
    add_test(a_segmentation_test test_segmentation)
endif(build)
