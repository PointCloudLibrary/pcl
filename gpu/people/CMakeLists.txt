set(SUBSYS_NAME gpu_people)
set(SUBSYS_PATH gpu/people)
set(SUBSYS_DESC "Point cloud people library")
set(SUBSYS_DEPS common io segmentation filters octree search kdtree gpu_containers gpu_utils visualization geometry)

set(build FALSE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} OFF)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
mark_as_advanced("BUILD_${SUBSYS_NAME}")

PCL_ADD_DOC(${SUBSYS_NAME})

if(build)
  #find_package(OpenCV QUIET)
  
  #find NPP
  unset(CUDA_npp_LIBRARY CACHE)
  find_cuda_helper_libs(npp)

  #Label_skeleton
  #FILE(GLOB srcs src/*.cpp src/*.h* src/label_skeleton/*.cpp)
  FILE(GLOB srcs src/*.cpp src/*.h*)  
  FILE(GLOB srcs_cuda src/cuda/*.cu src/cuda/*.h)  
  FILE(GLOB hdrs include/pcl/gpu/people/*.h*)
  
  source_group("Source files\\cuda" FILES ${srcs_cuda} )
  source_group("Source files" FILES ${srcs} )    

  include_directories(${VTK_INCLUDE_DIRS})
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/src/cuda)
  include_directories(${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

  if (UNIX OR APPLE)
    set (CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")
  endif()

  if(NOT UNIX OR APPLE)
    add_definitions(-DPCLAPI_EXPORTS)
  endif()

  CUDA_COMPILE(objs_cuda ${srcs_cuda} ${hdrs})

  set(LIB_NAME pcl_${SUBSYS_NAME})

  PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${hdrs} ${srcs_cuda} ${objs_cuda})
  target_link_libraries(${LIB_NAME} pcl_common pcl_io pcl_search pcl_sample_consensus pcl_gpu_utils pcl_gpu_containers ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY})
  PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

  # install include files
  PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_PATH} ${hdrs})

  add_subdirectory(tools)
endif(build)
