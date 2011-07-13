set(SUBSYS_NAME gpu_common)
set(SUBSYS_PATH cuda/gpu_common)
set(SUBSYS_DESC "Temporary GPU_common module. Weak CUDA dependency: a code that use this module requires CUDA Toolkit installed, but should be compiled without nvcc")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR(${SUBSYS_NAME} ${SUBSYS_PATH})
mark_as_advanced("BUILD_${SUBSYS_NAME}")

FILE(GLOB srcs src/*.cpp src/*.hpp)
FILE(GLOB incs include/pcl/gpu/common/*.hpp)

set(LIB_NAME pcl_${SUBSYS_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
PCL_ADD_SSE_FLAGS(${LIB_NAME})

PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
#list(APPEND EXT_DEPS CUDA)
PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${EXT_DEPS}" "" "" "")

# Install include files
PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_PATH} ${incs})
