set(SUBSYS_NAME cuda_sample_consensus)
set(SUBSYS_PATH cuda/sample_consensus)
set(SUBSYS_DESC "Point Cloud CUDA Sample Consensus library")
set(SUBSYS_DEPS cuda_common io common)

# ---[ Point Cloud Library - pcl/cuda/io

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
mark_as_advanced("BUILD_${SUBSYS_NAME}")
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")

if(NOT build)
  return()
endif()

set(srcs
  src/sac_model.cu
  src/sac_model_plane.cu
  src/sac_model_1point_plane.cu
  src/ransac.cu
  src/multi_ransac.cu
)

set(incs
  include/pcl/cuda/sample_consensus/sac.h
  include/pcl/cuda/sample_consensus/ransac.h
  include/pcl/cuda/sample_consensus/multi_ransac.h
  include/pcl/cuda/sample_consensus/sac_model.h
  include/pcl/cuda/sample_consensus/sac_model_plane.h
  include/pcl/cuda/sample_consensus/sac_model_1point_plane.h
)

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs})
target_link_libraries("${LIB_NAME}" pcl_cuda_features)

set(EXT_DEPS "")
#set(EXT_DEPS CUDA)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
