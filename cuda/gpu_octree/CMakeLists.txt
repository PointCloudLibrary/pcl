set(SUBSYS_NAME gpu_octree)
set(SUBSYS_PATH cuda/gpu_octree)
set(SUBSYS_DESC "Octree GPU")
set(SUBSYS_DEPS common gpu_common)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
mark_as_advanced("BUILD_${SUBSYS_NAME}")
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR(${SUBSYS_NAME} ${SUBSYS_PATH})

FILE(GLOB incs include/pcl/gpu/octree/*.hpp)
FILE(GLOB srcs src/*.cpp src/*.hpp src/*.cu)
source_group("Source Files" FILES ${srcs})

FILE(GLOB srcs_utils src/utils/*.hpp)
source_group("Source Files\\utils" FILES ${srcs_utils})

FILE(GLOB srcs_builder src/builder/*.hpp)
source_group("Source Files\\builder" FILES ${srcs_builder})

FILE(GLOB srcs_radius_search src/radius_search/*.hpp)
source_group("Source Files\\radius_search" FILES ${srcs_radius_search})

LIST(APPEND srcs ${srcs_utils} ${srcs_builder} ${srcs_radius_search})

set(LIB_NAME pcl_${SUBSYS_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/src/utils)
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs})
PCL_ADD_SSE_FLAGS(${LIB_NAME})
target_link_libraries(${LIB_NAME} pcl_gpu_common)

PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
#list(APPEND EXT_DEPS CUDA)
PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${EXT_DEPS}" "" "" "")

# Install include files
PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_PATH} ${incs})
