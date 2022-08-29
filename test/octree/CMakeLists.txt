set(SUBSYS_NAME tests_octree)
set(SUBSYS_DESC "Point cloud library octree module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS octree)

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(a_octree_test test_octree
              FILES test_octree.cpp
              LINK_WITH pcl_gtest pcl_common)
PCL_ADD_TEST(a_octree_iterator_test test_octree_iterator
              FILES test_octree_iterator.cpp
              LINK_WITH pcl_gtest pcl_common pcl_octree)
