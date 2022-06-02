set(SUBSYS_NAME aabb_tree)
set(SUBSYS_DESC "Geometry AABB tree library")
set(SUBSYS_DEPS common)
set(EXT_DEPS "")

option(BUILD_aabb_tree "Build AABB tree making use of CGAL" ON)

if (NOT BUILD_aabb_tree)
  return()
endif()

set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
find_package(CGAL)
if (NOT CGAL_FOUND)
  return()
endif()

set(build TRUE)

PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

set(srcs
  src/aabb_tree_cgal.cpp
)

set(incs
  "include/pcl/${SUBSYS_NAME}/aabb_tree.h"
  "include/pcl/${SUBSYS_NAME}/aabb_tree_cgal.h"
)

set(impl_incs
  "include/pcl/${SUBSYS_NAME}/impl/aabb_tree_cgal.hpp"
)

set(libs
  pcl_common
  CGAL::CGAL
)

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs} ${impl_incs})
target_link_libraries("${LIB_NAME}" ${libs})
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} EXT_DEPS ${EXT_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})
