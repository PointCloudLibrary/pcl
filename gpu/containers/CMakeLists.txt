set(SUBSYS_NAME gpu_containers)
set(SUBSYS_PATH gpu/containers)
set(SUBSYS_DESC "Containers with reference counting for GPU memory. This module can be compiled without Cuda Toolkit.")
set(SUBSYS_DEPS common gpu_utils)

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")
mark_as_advanced("BUILD_${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

file(GLOB srcs src/*.cpp src/*.cu src/*.hpp)
file(GLOB incs include/pcl/gpu/containers/*.h)
file(GLOB incs_impl include/pcl/gpu/containers/impl/*.hpp)

source_group("Header Files\\impl" FILES ${incs_impl})
list(APPEND incs ${incs_impl})

set(LIB_NAME "pcl_${SUBSYS_NAME}")
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs})
target_link_libraries(${LIB_NAME} pcl_common pcl_gpu_utils)

#Ensures that CUDA is added and the project can compile as it uses cuda calls.
set_source_files_properties(src/device_memory.cpp PROPERTIES LANGUAGE CUDA)

set(EXT_DEPS "")
#set(EXT_DEPS CUDA)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}/impl" ${incs_impl})
