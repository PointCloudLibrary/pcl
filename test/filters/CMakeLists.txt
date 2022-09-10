set(SUBSYS_NAME tests_filters)
set(SUBSYS_DESC "Point cloud library filters module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS filters)
set(OPT_DEPS io features segmentation)

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()


PCL_ADD_TEST(filters_grid_minimum test_filters_grid_minimum
             FILES test_grid_minimum.cpp
             LINK_WITH pcl_gtest pcl_common pcl_filters)

PCL_ADD_TEST(filters_morphological test_morphological
             FILES test_morphological.cpp
             LINK_WITH pcl_gtest pcl_common pcl_filters)

PCL_ADD_TEST(filters_functor test_filters_functor
             FILES test_functor_filter.cpp
             LINK_WITH pcl_gtest pcl_common pcl_filters)

PCL_ADD_TEST(filters_local_maximum test_filters_local_maximum
             FILES test_local_maximum.cpp
             LINK_WITH pcl_gtest pcl_common pcl_filters pcl_search pcl_octree)

PCL_ADD_TEST(filters_uniform_sampling test_uniform_sampling
             FILES test_uniform_sampling.cpp
             LINK_WITH pcl_gtest pcl_common pcl_filters)

PCL_ADD_TEST(filters_farthest_point_sampling test_farthest_point_sampling
             FILES test_farthest_point_sampling.cpp
             LINK_WITH pcl_gtest pcl_filters pcl_io
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bunny.pcd")

PCL_ADD_TEST(filters_convolution test_convolution
             FILES test_convolution.cpp
             LINK_WITH pcl_gtest pcl_filters)

PCL_ADD_TEST(filters_crop_hull test_crop_hull
             FILES test_crop_hull.cpp
             LINK_WITH pcl_gtest pcl_filters)

if(BUILD_io)
  PCL_ADD_TEST(filters_bilateral test_filters_bilateral
         FILES test_bilateral.cpp
         LINK_WITH pcl_gtest pcl_filters pcl_io pcl_kdtree
         ARGUMENTS "${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd")

  if(BUILD_features)
    PCL_ADD_TEST(filters_sampling test_filters_sampling
           FILES test_sampling.cpp
           LINK_WITH pcl_gtest pcl_filters pcl_io pcl_kdtree pcl_features
           ARGUMENTS "${PCL_SOURCE_DIR}/test/sac_plane_test.pcd" "${PCL_SOURCE_DIR}/test/cturtle.pcd")

    PCL_ADD_TEST(filters_model_outlier_removal test_model_outlier_removal
           FILES test_model_outlier_removal.cpp
           LINK_WITH pcl_gtest pcl_filters pcl_io pcl_sample_consensus pcl_kdtree pcl_features
           ARGUMENTS ${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd)

    if(BUILD_segmentation)
      PCL_ADD_TEST(filters_filters test_filters
             FILES test_filters.cpp
             LINK_WITH pcl_gtest pcl_filters pcl_io pcl_sample_consensus pcl_kdtree pcl_features pcl_segmentation
             ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd" "${PCL_SOURCE_DIR}/test/milk_cartoon_all_small_clorox.pcd")
      PCL_ADD_TEST(filters_clipper test_clipper
             FILES test_clipper.cpp
             LINK_WITH pcl_gtest pcl_filters)
    endif()
  endif()
endif()
