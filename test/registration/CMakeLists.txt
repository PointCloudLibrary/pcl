set(SUBSYS_NAME tests_registration)
set(SUBSYS_DESC "Point cloud library registration module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS registration)
set(OPT_DEPS io) # module does not depend on these

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

PCL_ADD_TEST(registration_warp_api test_warps
             FILES test_warps.cpp
             LINK_WITH pcl_gtest pcl_registration)

PCL_ADD_TEST(correspondence_estimation test_correspondence_estimation
             FILES test_correspondence_estimation.cpp
             LINK_WITH pcl_gtest pcl_registration pcl_features)

if(BUILD_io)
  PCL_ADD_TEST(a_registration_test test_registration
               FILES test_registration.cpp
               LINK_WITH pcl_gtest pcl_io pcl_registration pcl_features pcl_search pcl_kdtree
               ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd" "${PCL_SOURCE_DIR}/test/bun4.pcd" "${PCL_SOURCE_DIR}/test/milk_color.pcd")

  PCL_ADD_TEST(ndt test_ndt
               FILES test_ndt.cpp
               LINK_WITH pcl_gtest pcl_io pcl_registration
               ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd" "${PCL_SOURCE_DIR}/test/bun4.pcd")

  PCL_ADD_TEST(sac_ia test_sac_ia
               FILES test_sac_ia.cpp
               LINK_WITH pcl_gtest pcl_io pcl_registration
               ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd" "${PCL_SOURCE_DIR}/test/bun4.pcd")

  PCL_ADD_TEST(registration_api test_registration_api
               FILES test_registration_api.cpp test_registration_api_data.h
               LINK_WITH pcl_gtest pcl_io pcl_registration pcl_features pcl_kdtree
               ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd" "${PCL_SOURCE_DIR}/test/bun4.pcd")

  PCL_ADD_TEST(correspondence_rejectors test_correspondence_rejectors
               FILES test_correspondence_rejectors.cpp
               LINK_WITH pcl_gtest pcl_io pcl_registration pcl_features
               ARGUMENTS "${PCL_SOURCE_DIR}/test/bunny.pcd")

  PCL_ADD_TEST(fpcs_ia test_fpcs_ia
               FILES test_fpcs_ia.cpp test_fpcs_ia_data.h
               LINK_WITH pcl_gtest pcl_io pcl_registration pcl_features pcl_search pcl_kdtree
               ARGUMENTS "${PCL_SOURCE_DIR}/test/bun0.pcd" "${PCL_SOURCE_DIR}/test/bun4.pcd")

  PCL_ADD_TEST(kfpcs_ia test_kfpcs_ia
               FILES test_kfpcs_ia.cpp test_kfpcs_ia_data.h
               LINK_WITH pcl_gtest pcl_io pcl_registration pcl_features pcl_search pcl_kdtree
               ARGUMENTS "${PCL_SOURCE_DIR}/test/office1_keypoints.pcd" "${PCL_SOURCE_DIR}/test/office2_keypoints.pcd")
endif()
