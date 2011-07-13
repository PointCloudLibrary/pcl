if(BUILD_TESTS)						
	find_package(OpenCV)
	if (OpenCV_FOUND)
						
		get_filename_component(INC1 ${CMAKE_CURRENT_LIST_DIR}/../../../octree/include REALPATH)	
		#get_filename_component(INC2 ${CMAKE_CURRENT_LIST_DIR}/../../gpu_common/include REALPATH)	
		include_directories(${INC1} ${INC2} ${OpenCV_INCLUDE_DIRS})

		FILE(GLOB test_src *.cpp *.hpp)
		PCL_ADD_TEST(a_gpu_octree_test test_gpu_octree FILES ${test_src} LINK_WITH pcl_octree pcl_gpu_common pcl_gpu_octree)
			
		target_link_libraries(test_gpu_octree ${OpenCV_LIBS})		
		add_dependencies(test_gpu_octree pcl_gpu_octree pcl_octree pcl_gpu_common)
	endif()
endif(BUILD_TESTS)

