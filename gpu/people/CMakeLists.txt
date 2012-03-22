set(SUBSYS_NAME gpu_people)
set(SUBSYS_PATH gpu/people)
set(SUBSYS_DESC "Point cloud people library")
set(SUBSYS_DEPS common io segmentation filters octree search kdtree gpu_containers gpu_utils visualization)

set(build FALSE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} OFF)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC(${SUBSYS_NAME})

if(build)

  find_package(OpenCV QUIET)
    
  #Label_skeleton
  #FILE(GLOB srcs src/*.cpp src/*.h* src/label_skeleton/*.cpp)                   
  FILE(GLOB srcs src/*.cpp src/*.h*)                   
  FILE(GLOB_RECURSE hdrs include/pcl/people/*.h)                            
  FILE(GLOB srcs_cuda src/cuda/*.cu src/cuda/*.h)

  source_group("source files\\cuda" FILES ${srcs_cuda} )
  source_group("source files" FILES ${srcs} )	

  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/src/cuda)
  include_directories(${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
     
  if (UNIX OR APPLE)
	set (CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")			
  endif()
	
  if(NOT UNIX OR APPLE)
	add_definitions(-DPCLAPI_EXPORTS)
  endif()

  CUDA_COMPILE(objs_cuda ${srcs_cuda})

  set(LIB_NAME pcl_${SUBSYS_NAME})    

  PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${hdrs} ${srcs_cuda} ${objs_cuda})
  target_link_libraries(${LIB_NAME} pcl_search pcl_sample_consensus pcl_gpu_utils pcl_gpu_containers ${CUDA_CUDART_LIBRARY} ${OpenCV_LIBS})
  PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

  # install include files
  PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${hdrs})

  add_subdirectory(tools)
endif(build)
