set(SUBSYS_NAME tests_gpu_filters)
set(SUBSYS_DESC "Point cloud library gpu filters tests")
set(SUBSYS_DEPS common filters gpu_containers gpu_filters)

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(gpu_filters_voxel_grid test_gpu_voxel_grid FILES test_voxel_grid.cpp LINK_WITH pcl_gtest pcl_common pcl_filters pcl_gpu_containers pcl_gpu_filters)
