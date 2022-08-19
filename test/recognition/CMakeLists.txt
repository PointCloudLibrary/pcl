set(SUBSYS_NAME tests_recognition)
set(SUBSYS_DESC "Point cloud library recognition module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS recognition)
set(OPT_DEPS keypoints) # module does not depend on these

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(a_recognition_ism_test test_recognition_ism
             FILES test_recognition_ism.cpp
             LINK_WITH pcl_gtest pcl_io pcl_features
             ARGUMENTS "${PCL_SOURCE_DIR}/test/ism_train.pcd" "${PCL_SOURCE_DIR}/test/ism_test.pcd")

if(BUILD_keypoints)
  PCL_ADD_TEST(a_recognition_cg_test test_recognition_cg
               FILES test_recognition_cg.cpp
               LINK_WITH pcl_gtest pcl_common pcl_io pcl_kdtree pcl_features pcl_recognition pcl_keypoints
               ARGUMENTS "${PCL_SOURCE_DIR}/test/milk.pcd" "${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd")
endif()
