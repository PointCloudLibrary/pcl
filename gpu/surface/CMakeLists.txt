set(SUBSYS_NAME gpu_surface)
set(SUBSYS_PATH gpu/surface)
set(SUBSYS_DESC "Surface algorithms with GPU")
set(SUBSYS_DEPS common gpu_containers gpu_utils visualization geometry)

set(build FALSE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" OFF)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")
mark_as_advanced("BUILD_${SUBSYS_NAME}")

if(build) 
	FILE(GLOB incs include/pcl/gpu/surface/*.h)
	FILE(GLOB srcs src/*.cpp src/*.h*)
	source_group("Source Files" FILES ${srcs})

	FILE(GLOB utils src/utils/*.hpp)
	source_group("Source Files\\utils" FILES ${utils})

	FILE(GLOB cuda src/cuda/*.h* src/cuda/*.cu)
	source_group("Source Files\\cuda" FILES ${cuda})

	LIST(APPEND srcs ${utils} ${cuda})

	
	include_directories(${VTK_INCLUDE_DIRS})
	
	set(LIB_NAME "pcl_${SUBSYS_NAME}")
	include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/src/cuda")
	PCL_CUDA_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs})
	target_link_libraries("${LIB_NAME}" pcl_gpu_containers pcl_visualization)

	set(EXT_DEPS "")
	#set(EXT_DEPS CUDA)
	PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "${EXT_DEPS}" "" "" "")
	
	# Install include files
	PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
	
	add_subdirectory(test)
endif()
