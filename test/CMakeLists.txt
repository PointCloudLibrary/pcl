set(SUBSYS_NAME test)
set(SUBSYS_DESC "Point cloud library test files")
set(SUBSYS_DEPS common sample_consensus io kdtree features range_image filters keypoints surface registration segmentation)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
	include(FindGTest)
	enable_testing()
	find_package(GTest REQUIRED)
	include_directories(${GTEST_INCLUDE_DIRS})

	add_executable (test_narf test_narf.cpp)
	target_link_libraries (test_narf pcl_features ${GTEST_BOTH_LIBRARIES} flann)

	add_executable (test_vector_average test_vector_average.cpp)
	target_link_libraries (test_vector_average ${GTEST_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
	PCL_ADD_OPENMP_FLAGS(test_vector_average)

	add_executable (test_kdtree test_kdtree.cpp)
	target_link_libraries (test_kdtree pcl_kdtree ${GTEST_BOTH_LIBRARIES} ${Boost_SYSTEM_LIBRARY})
	PCL_ADD_OPENMP_FLAGS(test_kdtree)

	add_executable (test_common test_common.cpp)
	target_link_libraries(test_common ${GTEST_BOTH_LIBRARIES} ${Boost_THREAD_LIBRARY})
	PCL_ADD_OPENMP_FLAGS(test_common)

	add_executable (test_io test_io.cpp)
	target_link_libraries (test_io pcl_io ${GTEST_BOTH_LIBRARIES})

	add_executable (test_feature test_feature.cpp)
	PCL_ADD_OPENMP_FLAGS(test_feature)
	target_link_libraries (test_feature pcl_features pcl_kdtree pcl_io ${GTEST_BOTH_LIBRARIES})

	add_executable (test_sample_consensus test_sample_consensus.cpp)
	target_link_libraries (test_sample_consensus pcl_io pcl_sample_consensus pcl_kdtree pcl_features ${GTEST_BOTH_LIBRARIES})

	add_executable (test_filters test_filters.cpp)
	target_link_libraries (test_filters pcl_filters pcl_io pcl_sample_consensus pcl_kdtree ${GTEST_BOTH_LIBRARIES})

	add_executable (test_transforms test_transforms.cpp)
	target_link_libraries (test_transforms pcl_io ${GTEST_BOTH_LIBRARIES})

	add_executable (test_segmentation test_segmentation.cpp)
	target_link_libraries (test_segmentation pcl_io pcl_segmentation ${GTEST_BOTH_LIBRARIES})

	add_executable (test_iterators test_iterators.cpp)
	target_link_libraries (test_iterators pcl_io ${GTEST_BOTH_LIBRARIES})

	add_executable (test_registration test_registration.cpp)
	target_link_libraries (test_registration pcl_io pcl_registration pcl_features ${GTEST_BOTH_LIBRARIES})
	target_link_libraries (test_registration ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

	add_executable (test_keypoints test_keypoints.cpp)
	target_link_libraries (test_keypoints pcl_io pcl_kdtree pcl_filters ${GTEST_BOTH_LIBRARIES})

	add_executable (test_surface test_surface.cpp)
	target_link_libraries (test_surface pcl_io pcl_kdtree pcl_surface pcl_features ${GTEST_BOTH_LIBRARIES})

	add_executable (test_ii_normals test_ii_normals.cpp)
	target_link_libraries (test_ii_normals pcl_io pcl_features ${GTEST_BOTH_LIBRARIES})

	add_executable (test_mls test_mls.cpp)
	target_link_libraries (test_mls pcl_io pcl_kdtree pcl_surface pcl_filters ${GTEST_BOTH_LIBRARIES})

	add_test(a_kdtree_test test_kdtree)
	add_test(a_keypoints_test test_keypoints)
	add_test(a_narf_test test_narf)
	add_test(a_sample_consensus_test test_sample_consensus)
	add_test(a_transforms_test test_transforms)
	add_test(a_vector_average_test test_vector_average)
	add_test(a_filters_test test_filters)
	add_test(a_io_test test_io)
	add_test(a_mls_test test_mls)
	add_test(a_registration_test test_registration)
	add_test(a_common_test test_common)
	add_test(a_feature_test test_feature)
	add_test(a_ii_normals_test test_ii_normals)
	add_test(a_iterators_test test_iterators)
	add_test(a_segmentation_test test_segmentation)
	add_test(a_surface_test test_surface)
endif(build)
