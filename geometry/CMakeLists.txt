set(SUBSYS_NAME geometry)
set(SUBSYS_DESC "Point cloud geometry library")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

set(incs
  "include/pcl/${SUBSYS_NAME}/boost.h"
  "include/pcl/${SUBSYS_NAME}/eigen.h"
  "include/pcl/${SUBSYS_NAME}/get_boundary.h"
  "include/pcl/${SUBSYS_NAME}/line_iterator.h"
  "include/pcl/${SUBSYS_NAME}/mesh_base.h"
  "include/pcl/${SUBSYS_NAME}/mesh_circulators.h"
  "include/pcl/${SUBSYS_NAME}/mesh_conversion.h"
  "include/pcl/${SUBSYS_NAME}/mesh_elements.h"
  "include/pcl/${SUBSYS_NAME}/mesh_indices.h"
  "include/pcl/${SUBSYS_NAME}/mesh_io.h"
  "include/pcl/${SUBSYS_NAME}/mesh_traits.h"
  "include/pcl/${SUBSYS_NAME}/organized_index_iterator.h"
  "include/pcl/${SUBSYS_NAME}/planar_polygon.h"
  "include/pcl/${SUBSYS_NAME}/polygon_mesh.h"
  "include/pcl/${SUBSYS_NAME}/polygon_operations.h"
  "include/pcl/${SUBSYS_NAME}/quad_mesh.h"
  "include/pcl/${SUBSYS_NAME}/triangle_mesh.h"
)

set(impl_incs
  "include/pcl/${SUBSYS_NAME}/impl/polygon_operations.hpp"
)


set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(${LIB_NAME} INTERFACE)
target_include_directories(${LIB_NAME} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(${LIB_NAME} INTERFACE Boost::boost pcl_common)

PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS} HEADER_ONLY)

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})
