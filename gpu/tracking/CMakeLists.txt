set(SUBSYS_NAME gpu_tracking)
set(SUBSYS_PATH gpu/tracking)
set(SUBSYS_DESC "Tracking with GPU")
set(SUBSYS_DEPS common gpu_containers gpu_utils gpu_octree tracking search kdtree filters octree)

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" OFF)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")
mark_as_advanced("BUILD_${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

file(GLOB incs include/pcl/gpu/tracking/*.h)
file(GLOB srcs src/*.cpp src/*.h*)
source_group("Source Files" FILES ${srcs})

file(GLOB utils src/utils/*.hpp)
source_group("Source Files\\utils" FILES ${utils})

file(GLOB cuda src/cuda/*.h* src/cuda/*.cu)
source_group("Source Files\\cuda" FILES ${cuda})

list(APPEND srcs ${utils} ${cuda})

set(LIB_NAME "pcl_${SUBSYS_NAME}")
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs})
target_link_libraries(${LIB_NAME} pcl_gpu_containers)
target_include_directories(${LIB_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(EXT_DEPS "")
#set(EXT_DEPS CUDA)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
