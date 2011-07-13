if(BUILD_TESTS)		
		#set(SUBSYS_NAME gpu_perf_test)
		#set(SUBSYS_DESC "GPU OCtree test (algorithmic and perfomance)")
		#set(SUBSYS_DEPS octree gpu_common gpu_octree)
		
		#set(BUILD true)
		#PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
		#PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})
		
		#enable_testing()
		
		find_package(OpenCV REQUIRED)
		        	
		#get_filename_component(INC1 ${CMAKE_CURRENT_LIST_DIR}/../../../octree/include REALPATH)	
		#get_filename_component(INC2 ${CMAKE_CURRENT_LIST_DIR}/../../gpu_common/include REALPATH)	
		#include_directories(${INC1} ${INC2} ${OpenCV_INCLUDE_DIRS})

		PCL_ADD_TEST(a_gpu_octree_test test_gpu_octree FILES main.cpp LINK_WITH pcl_octree pcl_gpu_common pcl_gpu_octree)
		
		target_link_libraries(test_gpu_octree ${PCL_LIBRARIES} ${GTEST_BOTH_LIBRARIES} ${OpenCV_LIBS})		
		add_dependencies(test_gpu_octree pcl_gpu_octree pcl_octree pcl_gpu_common)
		
		message(STATUS "BOTH = ${GTEST_BOTH_LIBRARIES}")
		
		#opencv_contrib opencv_legacy opencv_objdetect opencv_calib3d opencv_features2d opencv_video opencv_highgui opencv_ml opencv_imgproc opencv_flann opencv_core
		
		#link_ros_libs (test_octree)
	
endif(BUILD_TESTS)

