set(SUBSYS_NAME people)
set(SUBSYS_DESC "Point cloud people library")
set(SUBSYS_DEPS common kdtree search sample_consensus filters io visualization geometry segmentation octree)

if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else()
  set(DEFAULT TRUE)
  set(REASON)
  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build NAME ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(NOT build)
  return()
endif()

set(incs
  "include/pcl/${SUBSYS_NAME}/ground_based_people_detection_app.h"
  "include/pcl/${SUBSYS_NAME}/head_based_subcluster.h"
  "include/pcl/${SUBSYS_NAME}/height_map_2d.h"
  "include/pcl/${SUBSYS_NAME}/person_classifier.h"
  "include/pcl/${SUBSYS_NAME}/person_cluster.h"
  "include/pcl/${SUBSYS_NAME}/hog.h"
)

set(impl_incs
  "include/pcl/${SUBSYS_NAME}/impl/ground_based_people_detection_app.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/head_based_subcluster.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/height_map_2d.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/person_classifier.hpp"
  "include/pcl/${SUBSYS_NAME}/impl/person_cluster.hpp"
)

set(srcs
  src/hog.cpp
)

set(LIB_NAME "pcl_${SUBSYS_NAME}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

PCL_ADD_LIBRARY(${LIB_NAME} COMPONENT ${SUBSYS_NAME} SOURCES ${srcs} ${incs} ${impl_incs})
target_link_libraries(${LIB_NAME} pcl_common pcl_filters pcl_kdtree pcl_sample_consensus pcl_segmentation pcl_visualization)

PCL_MAKE_PKGCONFIG(${LIB_NAME} COMPONENT ${SUBSYS_NAME} DESC ${SUBSYS_DESC} PCL_DEPS ${SUBSYS_DEPS})
# Install include files
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

#SET_TARGET_PROPERTIES("${LIB_NAME}" PROPERTIES LINKER_LANGUAGE CXX)

if(WITH_OPENNI)
  PCL_ADD_EXECUTABLE(pcl_ground_based_rgbd_people_detector COMPONENT ${SUBSYS_NAME} SOURCES apps/main_ground_based_people_detection.cpp BUNDLE)
  target_link_libraries(pcl_ground_based_rgbd_people_detector pcl_common pcl_kdtree pcl_search pcl_sample_consensus pcl_filters pcl_io pcl_visualization pcl_segmentation pcl_people)
endif()
