set(SUBSYS_NAME benchmarks)
set(SUBSYS_DESC "Point cloud library benchmarks")
set(SUBSYS_DEPS common filters features search kdtree io)
set(DEFAULT OFF)
set(build TRUE)
set(REASON "Disabled by default")
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
if(NOT build)
  return()
endif()

find_package(benchmark REQUIRED)
get_target_property(BenchmarkBuildType benchmark::benchmark TYPE)

add_custom_target(run_benchmarks)

PCL_ADD_BENCHMARK(features_normal_3d FILES features/normal_3d.cpp
                  LINK_WITH pcl_io pcl_search pcl_features
                  ARGUMENTS "${PCL_SOURCE_DIR}/test/table_scene_mug_stereo_textured.pcd"
                            "${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd")

PCL_ADD_BENCHMARK(filters_voxel_grid FILES filters/voxel_grid.cpp
                  LINK_WITH pcl_io pcl_filters
                  ARGUMENTS "${PCL_SOURCE_DIR}/test/table_scene_mug_stereo_textured.pcd"
                            "${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd")

