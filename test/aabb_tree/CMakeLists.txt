set(SUBSYS_NAME tests_aabb_tree)
set(SUBSYS_DESC "Point cloud library AABB tree module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS aabb_tree vtk)
set(OPT_DEPS io) # io is not a mandatory dependency in aabb_tree

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT (build AND BUILD_io))
  return()
endif()

PCL_ADD_TEST (aabb_tree_aabb_tree test_aabb_tree
              FILES test_aabb_tree.cpp
              LINK_WITH pcl_gtest pcl_aabb_tree pcl_io pcl_common
              ARGUMENTS "${PCL_SOURCE_DIR}/test/tum_rabbit.vtk")
