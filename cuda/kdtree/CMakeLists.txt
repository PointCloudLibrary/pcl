set(SUBSYS_NAME cuda_kdtree)
set(SUBSYS_PATH cuda/kdtree)
set(SUBSYS_DESC "Point cloud kd-tree CUDA library")
set(SUBSYS_DEPS common cuda_common kdtree)
set(EXT_DEPS flann)

# ---[ Point Cloud Library - pcl/cuda/kdtree

if (NOT TARGET flann::flann_cuda)
  return()
endif()

set(build TRUE)

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
mark_as_advanced("BUILD_${SUBSYS_NAME}")

PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})
PCL_SET_SUBSYS_INCLUDE_DIR("${SUBSYS_NAME}" "${SUBSYS_PATH}")

if(NOT build)
  return()
endif()

set(srcs
  src/kdtree_flann.cpp
)

set(incs
  include/pcl/${SUBSYS_PATH}/kdtree.h
  include/pcl/${SUBSYS_PATH}/kdtree_flann.h
)

set(impl_incs
  include/pcl/${SUBSYS_PATH}/impl/kdtree_flann.hpp
)

set(libs
  pcl_kdtree
  flann::flann_cuda
)

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
PCL_CUDA_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs} ${impl_incs})
target_link_libraries(${LIB_NAME} ${libs})
target_compile_definitions(${LIB_NAME} PUBLIC FLANN_USE_CUDA)

PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_PATH}/impl" ${impl_incs})
