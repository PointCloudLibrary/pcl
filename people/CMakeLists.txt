set(SUBSYS_NAME people)
set(SUBSYS_DESC "Point cloud people library")
set(SUBSYS_DEPS common kdtree search features sample_consensus filters io visualization geometry segmentation octree)

if(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
    set(VTK_USE_FILE "${VTK_USE_FILE}" CACHE INTERNAL "VTK_USE_FILE")
    include("${VTK_USE_FILE}")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
endif(NOT VTK_FOUND)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)
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
        src/hog.cpp)

    set(LIB_NAME "pcl_${SUBSYS_NAME}")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
    PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs})
    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")
    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})
    
    #SET_TARGET_PROPERTIES("${LIB_NAME}" PROPERTIES LINKER_LANGUAGE CXX)

    if(WITH_OPENNI)
        PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_ground_based_rgbd_people_detector "${SUBSYS_NAME}" apps/main_ground_based_people_detection.cpp)
        target_link_libraries(pcl_ground_based_rgbd_people_detector pcl_common pcl_kdtree pcl_search pcl_features pcl_sample_consensus pcl_filters pcl_io pcl_visualization pcl_segmentation pcl_people)
    endif()
endif(build)
