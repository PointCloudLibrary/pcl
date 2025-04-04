set(SUBSYS_NAME cuda_io)
set(SUBSYS_PATH cuda/io)
set(SUBSYS_DESC "Point cloud CUDA IO library")
set(SUBSYS_DEPS cuda_common io common)
set(SUBSYS_EXT_DEPS openni)

# ---[ Point Cloud Library - pcl/cuda/io

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" OFF)
mark_as_advanced("BUILD_${SUBSYS_NAME}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS ${SUBSYS_EXT_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")

if(NOT build)
  return()
endif()

set(srcs
  src/disparity_to_cloud.cu
  src/cloud_to_pcl.cpp
  src/host_device.cu
  src/extract_indices.cu
  src/debayering.cu
)

set(incs
  include/pcl/cuda/io/cloud_from_pcl.h
  include/pcl/cuda/io/cloud_to_pcl.h
  include/pcl/cuda/io/debayering.h
  include/pcl/cuda/io/disparity_to_cloud.h
  include/pcl/cuda/io/extract_indices.h
  include/pcl/cuda/io/host_device.h
  include/pcl/cuda/io/predicate.h
)

set(LIB_NAME "pcl_${SUBSYS_NAME}")
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs})
target_link_libraries(${LIB_NAME} pcl_common pcl_io pcl_cuda_common)

PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${SUBSYS_EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
