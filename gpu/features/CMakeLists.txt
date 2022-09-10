set(SUBSYS_NAME gpu_features)
set(SUBSYS_PATH gpu/features)
set(SUBSYS_DESC "Temporary GPU_common module. Weak CUDA dependency: a code that use this module requires CUDA Toolkit installed, but should be compiled without nvcc")
set(SUBSYS_DEPS common gpu_containers gpu_utils gpu_octree geometry)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")
mark_as_advanced("BUILD_${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

file(GLOB incs include/pcl/gpu/features/*.hpp)
file(GLOB srcs src/*.cpp src/*.hpp)
file(GLOB cuda src/*.cu)
file(GLOB dev_incs include/pcl/gpu/features/device/*.h*)
source_group("Header Files\\device" FILES ${dev_incs})
source_group("Source Files\\cuda" FILES ${cuda})
source_group("Source Files" FILES ${srcs})

file(GLOB srcs_utils src/utils/*.hpp)
source_group("Source Files\\utils" FILES ${srcs_utils})

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/src")
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs} ${cuda} ${srcs_utils}  ${dev_incs})
target_link_libraries("${LIB_NAME}" pcl_gpu_utils pcl_gpu_containers pcl_gpu_octree)

set(EXT_DEPS "")
#set(EXT_DEPS CUDA)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}/device" ${dev_incs})

add_subdirectory(test)
