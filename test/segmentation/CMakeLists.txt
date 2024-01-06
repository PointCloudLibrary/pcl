set(SUBSYS_NAME tests_segmentation)
set(SUBSYS_DESC "Point cloud library segmentation module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS segmentation)
set(OPT_DEPS) # module does not depend on these

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(random_walker test_random_walker
             FILES test_random_walker.cpp
             LINK_WITH pcl_gtest pcl_segmentation
             ARGUMENTS "${PCL_SOURCE_DIR}/test/segmentation/data")

PCL_ADD_TEST(a_segmentation_test test_segmentation
             FILES test_segmentation.cpp
             LINK_WITH pcl_gtest pcl_io pcl_segmentation pcl_features pcl_kdtree pcl_search pcl_common
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd" "${PCL_SOURCE_DIR}/test/car6.pcd" "${PCL_SOURCE_DIR}/test/colored_cloud.pcd")

PCL_ADD_TEST(test_non_linear test_non_linear
             FILES test_non_linear.cpp
             LINK_WITH pcl_gtest pcl_common pcl_io pcl_sample_consensus pcl_segmentation pcl_kdtree pcl_search
             ARGUMENTS "${PCL_SOURCE_DIR}/test/noisy_slice_displaced.pcd")
