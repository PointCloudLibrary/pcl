if(OPENNI_FOUND)
	if(NOT VTK_FOUND)
		set(DEFAULT FALSE)
		set(REASON "VTK was not found.")
	else(NOT VTK_FOUND)
		set(DEFAULT TRUE)
		set(REASON)
		set(VTK_USE_FILE ${VTK_USE_FILE} CACHE INTERNAL "VTK_USE_FILE")
		include (${VTK_USE_FILE})
		include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
	endif(NOT VTK_FOUND)

	## KINECT FUSION
	set(the_target kinfu_app)
	
	#FILE(GLOB srcs "*.c*")
	set(srcs kinfu_app.cpp capture.cpp)
	FILE(GLOB hdrs "*.h*")
	source_group("Source Files" FILES ${srcs} )

	message (STATUS "sources: " ${srcs})
	
	include_directories(${VTK_INCLUDE_DIRS})

	PCL_ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} ${srcs} ${hdrs})
	target_link_libraries(${the_target} pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu pcl_reconstruction)  
  
	if(OpenCV_FOUND)
		target_link_libraries(${the_target} ${OpenCV_LIBS})
	endif()

	## RECORD TSDFVOLUME
	set(the_target record_tsdfvolume)
	PCL_ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} record_tsdfvolume.cpp capture.cpp)
	target_link_libraries(${the_target} pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu pcl_reconstruction)  

endif(OPENNI_FOUND)
