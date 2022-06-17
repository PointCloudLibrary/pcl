set(SUBSYS_NAME tests_cuda_kdtree)
set(SUBSYS_DESC "Point cloud library kd-tree CUDA tests")
set(SUBSYS_DEPS common kdtree cuda_kdtree)
set(OPT_DEPS io) # io is not a mandatory dependency in kdtree

set(DEFAULT ON)
set(build TRUE)
set(REASON "")
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT (build AND BUILD_io))
  return()
endif()

PCL_ADD_TEST (cuda_kdtree test_cuda_kdtree
              FILES test_kdtree.cpp
              LINK_WITH pcl_gtest pcl_io pcl_common pcl_kdtree pcl_cuda_kdtree
              ARGUMENTS "${PCL_SOURCE_DIR}/test/sac_plane_test.pcd" "${PCL_SOURCE_DIR}/test/kdtree/kdtree_unit_test_results.xml")
