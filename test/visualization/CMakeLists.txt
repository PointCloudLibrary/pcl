set(SUBSYS_NAME tests_visualization)
set(SUBSYS_DESC "Point cloud library visualization module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS visualization)
set(OPT_DEPS features) # module does not depend on these

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

if(BUILD_features AND NOT UNIX OR (UNIX AND DEFINED ENV{DISPLAY}))
  PCL_ADD_TEST(a_visualization_test test_visualization
               FILES test_visualization.cpp
               LINK_WITH pcl_gtest pcl_io pcl_visualization pcl_features
               ARGUMENTS "${PCL_SOURCE_DIR}/test/bunny.pcd")
endif()
