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

set(incs
  "include/pcl/${SUBSYS_PATH}/device_format.hpp"
  "include/pcl/${SUBSYS_PATH}/octree.h"
  "include/pcl/${SUBSYS_PATH}/octree.hpp"
)
set(incs_impl
  "include/pcl/${SUBSYS_PATH}/impl/internal.hpp"
  "include/pcl/${SUBSYS_PATH}/impl/octree.hpp"
)
set(srcs
  "src/cuda/impl.cu"
  "src/cuda/approx_nsearch.cu"
  "src/cuda/bfrs.cu"
  "src/cuda/knn_search.cu"
  "src/cuda/octree_builder.cu"
  "src/cuda/octree_host.cu"
  "src/cuda/octree_iterator.hpp"
  "src/cuda/radius_search.cu"
  "src/utils/approx_nearest_utils.hpp"
  "src/utils/bitonic_sort.hpp"
  "src/utils/boxutils.hpp"
  "src/utils/copygen.hpp"
  "src/utils/emulation.hpp"
  "src/utils/morton.hpp"
  "src/utils/priority_octree_iterator.hpp"
  "src/utils/scan_block.hpp"
  "src/utils/warp_reduce.hpp"
)
source_group("Source Files" FILES ${srcs})
source_group("Header Files\\impl" FILES ${incs_impl})
list(APPEND incs ${incs_impl})

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/src/utils")
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs})
target_link_libraries("${LIB_NAME}" pcl_gpu_containers pcl_gpu_utils)

set(EXT_DEPS "")
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}/impl" ${incs_impl})
