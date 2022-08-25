set(SUBSYS_NAME tests_keypoints)
set(SUBSYS_DESC "Point cloud library keypoints module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS keypoints)
set(OPT_DEPS io) # module does not depend on these

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT (build AND BUILD_io))
  return()
endif()

PCL_ADD_TEST(keypoints_general test_keypoints
             FILES test_keypoints.cpp
             LINK_WITH pcl_gtest pcl_io pcl_kdtree pcl_filters pcl_keypoints
             ARGUMENTS "${PCL_SOURCE_DIR}/test/cturtle.pcd")

PCL_ADD_TEST(keypoints_iss_3d test_iss_3d
             FILES test_iss_3d.cpp
             LINK_WITH pcl_gtest pcl_keypoints pcl_io
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd")
