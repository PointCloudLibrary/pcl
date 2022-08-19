set(SUBSYS_NAME tests_outofcore)
set(SUBSYS_DESC "Point cloud library outofcore module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS outofcore)
set(OPT_DEPS) # module does not depend on these

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

include_directories(SYSTEM ${VTK_INCLUDE_DIRS})
PCL_ADD_TEST (outofcore_test test_outofcore
              FILES test_outofcore.cpp
              LINK_WITH pcl_gtest pcl_common pcl_io pcl_filters pcl_outofcore pcl_visualization)
