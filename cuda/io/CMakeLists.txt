set(SUBSYS_NAME cuda_io)
set(SUBSYS_PATH cuda/io)
set(SUBSYS_DESC "Point cloud CUDA IO library")
set(SUBSYS_DEPS cuda_common io common)

# ---[ Point Cloud Library - pcl/cuda/io

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
mark_as_advanced("BUILD_${SUBSYS_NAME}")
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR(${SUBSYS_NAME} ${SUBSYS_PATH})

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

set(LIB_NAME pcl_${SUBSYS_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
PCL_ADD_SSE_FLAGS(${LIB_NAME})

PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
#list(APPEND EXT_DEPS CUDA)
PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
    "${EXT_DEPS}" "" "" "")

# Install include files
PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_PATH} ${incs})
