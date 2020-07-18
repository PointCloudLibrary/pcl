set(SUBSYS_NAME gpu_people)
set(SUBSYS_PATH gpu/people)
set(SUBSYS_DESC "Point cloud people library")
set(SUBSYS_DEPS common features filters geometry gpu_containers gpu_utils io kdtree octree search segmentation surface visualization)

set(build FALSE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} OFF)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
mark_as_advanced("BUILD_${SUBSYS_NAME}")

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

REMOVE_VTK_DEFINITIONS()

#find NPP
unset(CUDA_npp_LIBRARY CACHE)
find_cuda_helper_libs(nppc)
find_cuda_helper_libs(npps)
if(CUDA_VERSION VERSION_GREATER "9.0" OR CUDA_VERSION VERSION_EQUAL "9.0")
  find_cuda_helper_libs(nppim)
  find_cuda_helper_libs(nppidei)
  set(CUDA_npp_LIBRARY ${CUDA_nppc_LIBRARY} ${CUDA_nppim_LIBRARY} ${CUDA_nppidei_LIBRARY} ${CUDA_npps_LIBRARY} CACHE STRING "npp library")
else()
  find_cuda_helper_libs(nppi)
  set(CUDA_npp_LIBRARY ${CUDA_nppc_LIBRARY} ${CUDA_nppi_LIBRARY} ${CUDA_npps_LIBRARY} CACHE STRING "npp library")
endif()

#Label_skeleton

file(GLOB srcs src/*.cpp src/*.h*)
file(GLOB srcs_cuda src/cuda/*.cu src/cuda/*.h src/cuda/nvidia/*.cu src/cuda/nvidia/*.hpp)
file(GLOB hdrs include/pcl/gpu/people/*.h*)
file(GLOB hdrs_cuda src/cuda/nvidia/*.h*)

source_group("Source files\\cuda" FILES ${srcs_cuda})
source_group("Source files" FILES ${srcs})

include_directories(
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/cuda"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/cuda/nvidia"
)

include_directories(SYSTEM
  ${VTK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

if(UNIX OR APPLE)
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")
endif()
if(NOT UNIX OR APPLE)
  add_definitions(-DPCLAPI_EXPORTS)
endif()

CUDA_COMPILE(objs_cuda ${srcs_cuda} ${hdrs})

set(LIB_NAME "pcl_${SUBSYS_NAME}")

PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${hdrs} ${srcs_cuda} ${objs_cuda})
target_link_libraries("${LIB_NAME}" pcl_common pcl_io pcl_search pcl_surface pcl_segmentation pcl_features pcl_sample_consensus pcl_gpu_utils pcl_gpu_containers "${CUDA_CUDART_LIBRARY}" ${CUDA_npp_LIBRARY})
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS})

# install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${hdrs} ${hdrs_cuda})

if(BUILD_tools)
  add_subdirectory(tools)
endif()
