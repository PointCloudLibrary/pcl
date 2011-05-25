set(SUBSYS_NAME cuda_features)
set(SUBSYS_PATH cuda/features)
set(SUBSYS_DESC "Point Cloud CUDA Feature Estimation library")
set(SUBSYS_DEPS cuda cuda_common io common)

# ---[ Point Cloud Library - pcl/cuda/io

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
mark_as_advanced("BUILD_${SUBSYS_NAME}")
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR(${SUBSYS_NAME} ${SUBSYS_PATH})

set(srcs
    src/normal_3d.cu
    )

set(incs 
    include/pcl/cuda/features/normal_3d.h
    )

set(LIB_NAME pcl_${SUBSYS_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} OPTIONS -arch sm_20)
PCL_ADD_SSE_FLAGS(${LIB_NAME})

PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
#list(APPEND EXT_DEPS CUDA)
PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
    "${EXT_DEPS}" "" "" "")

# Install include files
PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_PATH} ${incs})


