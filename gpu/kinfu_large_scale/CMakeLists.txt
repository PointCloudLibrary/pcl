set(SUBSYS_NAME gpu_kinfu_large_scale)
set(SUBSYS_PATH gpu/kinfu_large_scale)
set(SUBSYS_DESC "Kinect Fusion implementation, with volume shifting")
set(SUBSYS_DEPS common visualization io gpu_containers gpu_utils geometry search octree filters kdtree features surface)

set(build FALSE)

# OpenNI found?
if(NOT OPENNI_FOUND OR NOT BUILD_OPENNI)
    set(DEFAULT FALSE)
    set(REASON "OpenNI was not found or was disabled by the user.")
else()
    set(DEFAULT TRUE)
    set(REASON)
endif()

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")
mark_as_advanced("BUILD_${SUBSYS_NAME}")

if (build)
	FILE(GLOB incs include/pcl/gpu/kinfu_large_scale/*.h*)
	FILE(GLOB impl_incs include/pcl/gpu/kinfu_large_scale/impl/*.h*)
	FILE(GLOB srcs src/*.cpp src/*.h*)
	FILE(GLOB cuda src/cuda/*.cu src/cuda/*.h*)
	#FILE(GLOB tsdf src/cuda/tsdf_volume.cu)
	#FILE(GLOB est src/cuda/estimate_tranform.cu)
			
	source_group("Source Files\\cuda" FILES ${cuda} )    
	source_group("Source Files" FILES ${srcs} )		
	
	set(LIB_NAME "pcl_${SUBSYS_NAME}")
	include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/src" ${CUDA_INCLUDE_DIRS})

	if (UNIX OR APPLE)
		set (CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")			
	endif()
	
	if(NOT UNIX OR APPLE)
		add_definitions(-DPCLAPI_EXPORTS)
	endif()
	
	#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-gencode;arch=compute_11,code=compute_11;-gencode;arch=compute_12,code=compute_12")
	
	if(TRUE)
		#list(REMOVE_ITEM cuda ${est})
		#CUDA_COMPILE(est_objs ${est})		
				   
		set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")		
		CUDA_COMPILE(cuda_objs ${cuda})		
		
		#LIST(APPEND cuda ${est})
		#LIST(APPEND cuda_objs ${est_objs})
		
	else()		
		list(REMOVE_ITEM cuda ${tsdf})
		CUDA_COMPILE(cuda_objs ${cuda})		
		set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "--ftz=true;--prec-div=false;--prec-sqrt=false") 
		CUDA_COMPILE(tsdf_obj ${tsdf})
	
		LIST(APPEND cuda ${tsdf})
		LIST(APPEND cuda_objs ${tsdf_obj})
	endif()
	 
	PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs} ${cuda} ${cuda_objs})
	target_link_libraries("${LIB_NAME}" pcl_common pcl_io pcl_gpu_utils pcl_gpu_containers pcl_gpu_octree pcl_octree pcl_filters)
			
	set(EXT_DEPS "")
	#set(EXT_DEPS CUDA)
	PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "${EXT_DEPS}" "" "" "")

	# Install include files
	PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
	
	add_subdirectory(test)
	add_subdirectory(tools)
endif()


