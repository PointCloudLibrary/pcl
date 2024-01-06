set(SUBSYS_NAME tests_kdtree)
set(SUBSYS_DESC "Point cloud library kdtree module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS kdtree)
set(OPT_DEPS io) # io is not a mandatory dependency in kdtree

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT (build AND BUILD_io))
  return()
endif()

PCL_ADD_TEST (kdtree_kdtree test_kdtree
              FILES test_kdtree.cpp
              LINK_WITH pcl_gtest pcl_kdtree pcl_io pcl_common
              ARGUMENTS "${PCL_SOURCE_DIR}/test/sac_plane_test.pcd" "${PCL_SOURCE_DIR}/test/kdtree/kdtree_unit_test_results.xml")
