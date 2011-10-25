IF(TRUE)

	set(SUBSYS_NAME gpu_kinfu)
	set(SUBSYS_PATH cuda/gpu_kinfu)
	set(SUBSYS_DESC "Kinnect Fusiion implementation")
	set(SUBSYS_DEPS common gpu_containers gpu_utils gpu_features visualization)

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

	set(build FALSE)
	PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
	PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
	PCL_SET_SUBSYS_INCLUDE_DIR(${SUBSYS_NAME} ${SUBSYS_PATH})
	mark_as_advanced("BUILD_${SUBSYS_NAME}")
	

	if (build)
		FILE(GLOB incs include/pcl/gpu/kinfu/*.hpp)
		FILE(GLOB srcs src/*.cpp src/*.hpp)
		FILE(GLOB cuda src/cuda/*.cu src/cuda/*.h*)
		FILE(GLOB dev_incs include/pcl/gpu/features/device/*.h*)  
		source_group("Header Files\\device" FILES ${dev_incs})
		source_group("Source Files\\cuda" FILES ${cuda} )    
		source_group("Source Files" FILES ${srcs} )

		FILE(GLOB srcs_utils src/utils/*.hpp)
		source_group("Source Files\\utils" FILES ${srcs_utils})

		set(LIB_NAME pcl_${SUBSYS_NAME})
		include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
		PCL_CUDA_ADD_EXECUTABLE(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${cuda} ${srcs_utils}  ${dev_incs})
		PCL_ADD_SSE_FLAGS(${LIB_NAME})
		target_link_libraries(${LIB_NAME} pcl_gpu_utils pcl_gpu_containers pcl_gpu_octree pcl_visualization)
		set_target_properties(${LIB_NAME} PROPERTIES FOLDER "Libraries")
		
    target_link_libraries(${LIB_NAME} ${OPENNI_LIBRARIES} pcl_visualization)
    #vtkCommon vtkWidgets vtkHybrid)

		set(EXT_DEPS "")
		#set(EXT_DEPS CUDA)
		PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "${EXT_DEPS}" "" "" "")

		# Install include files
		PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_PATH} ${incs} ${dev_incs})
		
		add_subdirectory(test)
	endif()

endif()
