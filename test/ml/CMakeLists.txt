set(SUBSYS_NAME tests_ml)
set(SUBSYS_DESC "Point cloud library ml module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS ml)

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(ml_kmeans test_ml_kmeans FILES test_kmeans.cpp LINK_WITH pcl_gtest pcl_common pcl_ml)
