set(SUBSYS_NAME gpu_utils)
set(SUBSYS_PATH gpu/utils)
set(SUBSYS_DESC "Device layer functions.")
set(SUBSYS_DEPS common gpu_containers)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")
mark_as_advanced("BUILD_${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

file(GLOB incs include/pcl/gpu/utils/*.h*)
file(GLOB srcs src/*.cpp src/*.cu src/*.h*)
file(GLOB dev_incs include/pcl/gpu/utils/device/*.h*)
source_group("Header Files\\device" FILES ${dev_incs})
source_group("Source Files" FILES ${srcs})

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${dev_incs} ${incs} ${srcs})
target_link_libraries("${LIB_NAME}" pcl_gpu_containers)

set(EXT_DEPS "")
#set(EXT_DEPS CUDA)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}/device" ${dev_incs})
