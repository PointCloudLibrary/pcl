set(SUBSYS_NAME gpu_octree)
set(SUBSYS_PATH gpu/octree)
set(SUBSYS_DESC "Octree GPU")
set(SUBSYS_DEPS common gpu_containers gpu_utils)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
mark_as_advanced("BUILD_${SUBSYS_NAME}")
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")

if(NOT build)
  return()
endif()

file(GLOB incs include/pcl/gpu/octree/*.hpp)
file(GLOB srcs src/*.cpp src/*.hpp src/*.cu)
source_group("Source Files" FILES ${srcs})

file(GLOB utils src/utils/*.hpp)
source_group("Source Files\\utils" FILES ${utils})

file(GLOB cuda src/cuda/*.hpp src/cuda/*.cu)
source_group("Source Files\\cuda" FILES ${cuda})

list(APPEND srcs ${utils} ${cuda})

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/src/utils")
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs})
target_link_libraries("${LIB_NAME}" pcl_gpu_containers)

set(EXT_DEPS "")
#set(EXT_DEPS CUDA)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
