set(SUBSYS_NAME tests_search)
set(SUBSYS_DESC "Point cloud library search module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS search)
set(OPT_DEPS io)

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(kdtree_search test_kdtree_search
             FILES test_kdtree.cpp
             LINK_WITH pcl_gtest pcl_search pcl_kdtree)

PCL_ADD_TEST(flann_search test_flann_search
             FILES test_flann_search.cpp
             LINK_WITH pcl_gtest pcl_search pcl_kdtree)

PCL_ADD_TEST(organized_neighbor test_organized_search
             FILES test_organized.cpp
             LINK_WITH pcl_gtest pcl_search pcl_kdtree)

PCL_ADD_TEST(octree_search test_octree_search
             FILES test_octree.cpp
             LINK_WITH pcl_gtest pcl_search pcl_octree pcl_common)

if(BUILD_io)
  PCL_ADD_TEST(search test_search
               FILES test_search.cpp
               LINK_WITH pcl_gtest pcl_search pcl_io pcl_kdtree
               ARGUMENTS "${PCL_SOURCE_DIR}/test/table_scene_mug_stereo_textured.pcd")

  PCL_ADD_TEST(organized_index test_organized_index
          FILES test_organized_index.cpp
          LINK_WITH pcl_gtest pcl_search pcl_common pcl_io
          ARGUMENTS "${PCL_SOURCE_DIR}/test/office1.pcd")
endif()
