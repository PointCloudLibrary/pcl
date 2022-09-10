set(SUBSYS_NAME tests_2d)
set(SUBSYS_DESC "Point cloud library 2d module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS 2d)

set(OPT_DEPS)

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(test_2d test_2d FILES test_2d.cpp
             LINK_WITH pcl_io pcl_gtest
             ARGUMENTS "${PCL_SOURCE_DIR}/test/2d/lena.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/gauss_smooth.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/erosion.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/dilation.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/opening.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/closing.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/erosion_binary.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/dilation_binary.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/opening_binary.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/closing_binary.pcd"
                       "${PCL_SOURCE_DIR}/test/2d/canny.pcd")

PCL_ADD_TEST(test_2d_keypoint_instantiation_with_precompile test_2d_keypoint_instantiation_with_precompile FILES keypoint_instantiation.cpp LINK_WITH pcl_gtest pcl_common)

PCL_ADD_TEST(test_2d_keypoint_instantiation_without_precompile test_2d_keypoint_instantiation_without_precompile FILES keypoint_instantiation.cpp LINK_WITH pcl_gtest pcl_common)
target_compile_definitions(test_2d_keypoint_instantiation_without_precompile PUBLIC PCL_NO_PRECOMPILE)
