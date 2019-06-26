set(SUBSYS_NAME gpu_containers)
set(SUBSYS_PATH gpu/containers)
set(SUBSYS_DESC "Containers with reference counting for GPU memory. This module can be compiled without Cuda Toolkit.")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")
mark_as_advanced("BUILD_${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

file(GLOB srcs src/*.cpp src/*.hpp)
file(GLOB incs include/pcl/gpu/containers/*.h)
file(GLOB incs_impl include/pcl/gpu/containers/impl/*.hpp)

source_group("Header Files\\impl" FILES ${incs_impl})
list(APPEND incs ${incs_impl})

get_filename_component(UTILS_INC "${CMAKE_CURRENT_SOURCE_DIR}/../utils/include" REALPATH)

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include" ${UTILS_INC})
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs})

set(EXT_DEPS "")
#set(EXT_DEPS CUDA)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}/impl" ${incs_impl})
