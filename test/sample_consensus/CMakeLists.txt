set(SUBSYS_NAME tests_sample_consensus)
set(SUBSYS_DESC "Point cloud library sample_consensus module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS sample_consensus)
set(OPT_DEPS io)

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(sample_consensus test_sample_consensus
             FILES test_sample_consensus.cpp
             LINK_WITH pcl_gtest pcl_sample_consensus pcl_search pcl_common)

if(BUILD_io)
  PCL_ADD_TEST(sample_consensus_plane_models test_sample_consensus_plane_models
               FILES test_sample_consensus_plane_models.cpp
               LINK_WITH pcl_gtest pcl_io pcl_sample_consensus
               ARGUMENTS "${PCL_SOURCE_DIR}/test/sac_plane_test.pcd")
endif()

PCL_ADD_TEST(sample_consensus_quadric_models test_sample_consensus_quadric_models
             FILES test_sample_consensus_quadric_models.cpp
             LINK_WITH pcl_gtest pcl_sample_consensus)

PCL_ADD_TEST(sample_consensus_line_models test_sample_consensus_line_models
             FILES test_sample_consensus_line_models.cpp
             LINK_WITH pcl_gtest pcl_sample_consensus)
