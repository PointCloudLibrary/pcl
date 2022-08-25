set(SUBSYS_NAME tests_surface)
set(SUBSYS_DESC "Point cloud library surface module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS surface)
set(OPT_DEPS io features sample_consensus filters) # module does not depend on these

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT (build AND BUILD_io AND BUILD_features))
  return()
endif()

PCL_ADD_TEST(surface_marching_cubes test_marching_cubes
             FILES test_marching_cubes.cpp
             LINK_WITH pcl_gtest pcl_io pcl_kdtree pcl_surface pcl_features pcl_search
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd")
PCL_ADD_TEST(surface_moving_least_squares test_moving_least_squares
             FILES test_moving_least_squares.cpp
             LINK_WITH pcl_gtest pcl_io pcl_kdtree pcl_surface pcl_features pcl_search
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd")
PCL_ADD_TEST(surface_gp3 test_gp3
             FILES test_gp3.cpp
             LINK_WITH pcl_gtest pcl_io pcl_kdtree pcl_surface pcl_features pcl_search
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd")
PCL_ADD_TEST(surface_organized_fast_mesh test_organized_fast_mesh
             FILES test_organized_fast_mesh.cpp
             LINK_WITH pcl_gtest pcl_io pcl_kdtree pcl_surface pcl_features pcl_search
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd")
PCL_ADD_TEST(surface_grid_projection test_grid_projection
             FILES test_grid_projection.cpp
             LINK_WITH pcl_gtest pcl_io pcl_kdtree pcl_surface pcl_features pcl_search
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd")
PCL_ADD_TEST(surface_ear_clipping test_ear_clipping
             FILES test_ear_clipping.cpp
             LINK_WITH pcl_gtest pcl_io pcl_kdtree pcl_surface pcl_features pcl_search
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd")
PCL_ADD_TEST(surface_poisson test_poisson
             FILES test_poisson.cpp
             LINK_WITH pcl_gtest pcl_io pcl_kdtree pcl_surface pcl_features
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd")

if(QHULL_FOUND)
  PCL_ADD_TEST(surface_convex_hull test_convex_hull
               FILES test_convex_hull.cpp
               LINK_WITH pcl_gtest pcl_io pcl_kdtree pcl_surface pcl_features pcl_filters pcl_search
               ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd")
  if(BUILD_sample_consensus AND BUILD_filters)
    PCL_ADD_TEST(surface_concave test_concave_hull
                 FILES test_concave_hull.cpp
                 LINK_WITH pcl_gtest pcl_io pcl_kdtree pcl_surface pcl_features pcl_filters pcl_search pcl_sample_consensus
                 ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd")
  endif()
endif()
