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

	FILE(GLOB hdrs "*.h*")
	include_directories(${VTK_INCLUDE_DIRS})


  ## STANDALONE TEXTURE MAPPING
	set(the_target pcl_kinfu_largeScale_texture_output)
	set(srcs standalone_texture_mapping.cpp )

	source_group("Source Files" FILES ${srcs} )

	PCL_ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} ${srcs} ${hdrs})
	target_link_libraries(${the_target} pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu_large_scale pcl_kdtree pcl_features pcl_surface)

	## KINECT FUSION
	set(the_target pcl_kinfu_largeScale)
	set(srcs kinfuLS_app.cpp capture.cpp evaluation.cpp)

	source_group("Source Files" FILES ${srcs} )

	PCL_ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} ${srcs} ${hdrs})
	target_link_libraries(${the_target} pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu_large_scale pcl_octree)  
  
  ## STANDALONE MARCHING CUBES
	set(the_target pcl_kinfu_largeScale_mesh_output)
	set(srcs process_kinfu_large_scale_output.cpp)

	PCL_ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} ${srcs} ${hdrs})
	target_link_libraries(${the_target} pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu_large_scale pcl_filters)
        
	## WORLD MODEL TEST
	#set(the_target world_model_test)
	#set(srcs world_model_test.cpp)

	#source_group("Source Files" FILES ${srcs} )

	#PCL_ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} ${srcs} ${hdrs})
	#target_link_libraries(${the_target} pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu pcl_octree)  

	## RECORD TSDFVOLUME (disabled)
	# set(the_target record_tsdfvolume)
	# set(srcs record_tsdfvolume.cpp capture.cpp)
	#
	# PCL_ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} ${srcs} ${hdrs})
	# target_link_libraries(${the_target} pcl_common pcl_io ${OPENNI_LIBRARIES} pcl_visualization pcl_gpu_kinfu)  

	## KINECT FUSION SIMULATION - DISABLED FOR NOW
	#find_package(GLEW)
	#if(GLEW_FOUND)
	#	find_package(GLUT)
	#	if(GLUT_FOUND)
	#		include_directories(${GLUT_INCLUDE_DIR} ${GLEW_INCLUDE_DIR})
	#		## KINECT FUSION SIMULATION
	#		set(the_target kinfu_app_sim)
	#		set(srcs kinfu_app_sim.cpp capture.cpp evaluation.cpp)
	#
	#		source_group("Source Files" FILES ${srcs} )
	#		PCL_ADD_EXECUTABLE(${the_target} ${SUBSYS_NAME} ${srcs} ${hdrs})
	#		target_link_libraries(${the_target}  pcl_common pcl_io 
  	#			pcl_visualization pcl_kdtree  pcl_simulation  
	#	 		${OPENNI_LIBRARIES} pcl_gpu_kinfu
	#	  		${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})
 	#
	#	endif(GLUT_FOUND)
	#endif (GLEW_FOUND)

endif(OPENNI_FOUND)
