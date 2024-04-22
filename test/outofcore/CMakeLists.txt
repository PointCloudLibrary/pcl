set(SUBSYS_NAME tests_outofcore)
set(SUBSYS_DESC "Point cloud library outofcore module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS outofcore)
set(OPT_DEPS) # module does not depend on these

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST (outofcore_test test_outofcore
              FILES test_outofcore.cpp
              LINK_WITH pcl_gtest pcl_common pcl_io pcl_filters pcl_outofcore pcl_visualization)
