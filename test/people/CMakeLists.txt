set(SUBSYS_NAME tests_people)
set(SUBSYS_DESC "Point cloud library people module unit tests")
PCL_SET_TEST_DEPENDENCIES(SUBSYS_DEPS people)
set(OPT_DEPS) # module does not depend on these

set(DEFAULT ON)
set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS ${OPT_DEPS})

if(NOT build)
  return()
endif()

include_directories(SYSTEM ${VTK_INCLUDE_DIRS})
PCL_ADD_TEST(a_people_detection_test test_people_detection
             FILES test_people_groundBasedPeopleDetectionApp.cpp
             LINK_WITH pcl_gtest pcl_common pcl_io pcl_kdtree pcl_search pcl_features pcl_sample_consensus pcl_filters pcl_segmentation pcl_people
             ARGUMENTS "${PCL_SOURCE_DIR}/people/data/trainedLinearSVMForPeopleDetectionWithHOG.yaml" "${PCL_SOURCE_DIR}/test/five_people.pcd")
