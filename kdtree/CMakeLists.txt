set(SUBSYS_NAME kdtree)
set(SUBSYS_DESC "Point cloud kd-tree library")
set(SUBSYS_DEPS common)
set(EXT_DEPS flann)

option(BUILD_kdtree_nanoflann "Build kd-tree making use of nanoflann" ON)

if(BUILD_kdtree_nanoflann)
  find_package(nanoflann 1.4.2)
  if (nanoflann_FOUND)
    set(HAVE_nanoflann ON PARENT_SCOPE)
  endif()
endif()

set(build TRUE)

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

set(srcs
  src/kdtree_flann.cpp
)

set(incs
  "include/pcl/${SUBSYS_NAME}/kdtree.h"
  "include/pcl/${SUBSYS_NAME}/io.h"
  "include/pcl/${SUBSYS_NAME}/kdtree_flann.h"
)

set(impl_incs
  "include/pcl/${SUBSYS_NAME}/impl/io.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/kdtree_flann.hpp"
)

set(libs
  pcl_common
  FLANN::FLANN
)

if(HAVE_nanoflann)
  list(APPEND srcs src/kdtree_nanoflann.cpp)
  list(APPEND incs include/pcl/${SUBSYS_NAME}/kdtree_nanoflann.h)
  list(APPEND impl_incs include/pcl/${SUBSYS_NAME}/impl/kdtree_nanoflann.hpp)
  list(APPEND libs nanoflann::nanoflann)
endif()

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs} ${impl_incs})
target_link_libraries("${LIB_NAME}" ${libs})
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})
