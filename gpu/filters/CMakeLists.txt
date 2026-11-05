set(SUBSYS_NAME gpu_filters)
set(SUBSYS_PATH gpu/filters)
set(SUBSYS_DESC "Point cloud GPU filters library")
set(SUBSYS_DEPS common gpu_containers gpu_utils)

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")
mark_as_advanced("BUILD_${SUBSYS_NAME}")

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

file(GLOB incs include/pcl/gpu/filters/*.h*)
file(GLOB srcs src/*.cpp src/*.hpp src/*.cu)
source_group("Source Files" FILES ${srcs})

set(LIB_NAME "pcl_${SUBSYS_NAME}")
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs})
target_link_libraries("${LIB_NAME}" pcl_common pcl_gpu_containers pcl_gpu_utils)
target_include_directories(${LIB_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(EXT_DEPS "")
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
