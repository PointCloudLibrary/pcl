set(SUBSYS_NAME gpu_people)
set(SUBSYS_PATH gpu/people)
set(SUBSYS_DESC "Point cloud people library")
set(SUBSYS_DEPS common features filters geometry gpu_containers gpu_utils io kdtree octree search segmentation surface visualization)

set(build FALSE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} OFF)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
mark_as_advanced("BUILD_${SUBSYS_NAME}")

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)
  #find_package(OpenCV QUIET)
  
  #find NPP
  unset(CUDA_npp_LIBRARY CACHE)
  if(${CUDA_VERSION} VERSION_LESS "5.5")
	find_cuda_helper_libs(npp)
  else()
    find_cuda_helper_libs(nppc)
    find_cuda_helper_libs(nppi)
    find_cuda_helper_libs(npps)

    set(CUDA_npp_LIBRARY ${CUDA_nppc_LIBRARY} ${CUDA_nppi_LIBRARY} ${CUDA_npps_LIBRARY} CACHE STRING "npp library")
  endif()

  #Label_skeleton

  FILE(GLOB srcs src/*.cpp src/*.h*)
  FILE(GLOB srcs_cuda src/cuda/*.cu src/cuda/*.h src/cuda/nvidia/*.cu src/cuda/nvidia/*.hpp)
  #FILE(GLOB srcs_cuda src/cuda/*.cu src/cuda/*.h src/cuda/nvidia/*.cu src/cuda/nvidia/*.hpp)
  FILE(GLOB hdrs include/pcl/gpu/people/*.h*)
  FILE(GLOB hdrs_cuda src/cuda/nvidia/*.h*)

  source_group("Source files\\cuda" FILES ${srcs_cuda} )
  source_group("Source files" FILES ${srcs} )    

  include_directories(${VTK_INCLUDE_DIRS}
                      "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/src/cuda" "${CMAKE_CURRENT_SOURCE_DIR}/src/cuda/nvidia")
  #include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/src/cuda")
  include_directories(${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

  if (UNIX OR APPLE)
    set (CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")
  endif()
  if(NOT UNIX OR APPLE)
    add_definitions(-DPCLAPI_EXPORTS)
  endif()

  CUDA_COMPILE(objs_cuda ${srcs_cuda} ${hdrs})

  set(LIB_NAME "pcl_${SUBSYS_NAME}")

  PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${hdrs} ${srcs_cuda} ${objs_cuda})
  target_link_libraries("${LIB_NAME}" pcl_common pcl_io pcl_search pcl_surface pcl_segmentation pcl_features pcl_sample_consensus pcl_gpu_utils pcl_gpu_containers "${CUDA_CUDART_LIBRARY}" ${CUDA_npp_LIBRARY})
  PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

  # install include files
  PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${hdrs} ${hdrs_cuda})

  add_subdirectory(tools)
endif(build)
