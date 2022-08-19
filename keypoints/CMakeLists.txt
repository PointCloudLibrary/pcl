set(SUBSYS_NAME keypoints)
set(SUBSYS_DESC "Point cloud keypoints library")
set(SUBSYS_DEPS common search kdtree octree features filters)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

set(srcs
  src/narf_keypoint.cpp
  src/sift_keypoint.cpp
  src/smoothed_surfaces_keypoint.cpp
  src/harris_3d.cpp
  src/harris_6d.cpp
  src/agast_2d.cpp
  src/susan.cpp
  src/iss_3d.cpp
  src/brisk_2d.cpp
  src/trajkovic_2d.cpp
  src/trajkovic_3d.cpp
)

set(incs
  "include/pcl/${SUBSYS_NAME}/keypoint.h"
  "include/pcl/${SUBSYS_NAME}/narf_keypoint.h"
  "include/pcl/${SUBSYS_NAME}/sift_keypoint.h"
  "include/pcl/${SUBSYS_NAME}/uniform_sampling.h"
  "include/pcl/${SUBSYS_NAME}/smoothed_surfaces_keypoint.h"
  "include/pcl/${SUBSYS_NAME}/agast_2d.h"
  "include/pcl/${SUBSYS_NAME}/harris_2d.h"
  "include/pcl/${SUBSYS_NAME}/harris_3d.h"
  "include/pcl/${SUBSYS_NAME}/harris_6d.h"
  "include/pcl/${SUBSYS_NAME}/susan.h"
  "include/pcl/${SUBSYS_NAME}/iss_3d.h"
  "include/pcl/${SUBSYS_NAME}/brisk_2d.h"
  "include/pcl/${SUBSYS_NAME}/trajkovic_2d.h"
  "include/pcl/${SUBSYS_NAME}/trajkovic_3d.h"
)

set(impl_incs
  "include/pcl/${SUBSYS_NAME}/impl/keypoint.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/sift_keypoint.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/smoothed_surfaces_keypoint.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/agast_2d.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/harris_2d.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/harris_3d.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/harris_6d.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/susan.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/iss_3d.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/brisk_2d.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/trajkovic_2d.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/trajkovic_3d.hpp"
)

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs} ${impl_incs})
target_link_libraries("${LIB_NAME}" pcl_features pcl_filters)
PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS})

# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})
