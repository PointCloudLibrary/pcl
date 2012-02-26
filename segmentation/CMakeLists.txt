set(SUBSYS_NAME segmentation)
set(SUBSYS_DESC "Point cloud segmentation library")
set(SUBSYS_DEPS common search sample_consensus kdtree octree)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC(${SUBSYS_NAME})

if(build)
    set(srcs src/extract_clusters.cpp
        src/extract_polygonal_prism_data.cpp
        src/sac_segmentation.cpp
        src/seeded_hue_segmentation.cpp
        src/segment_differences.cpp
        #src/region_growing.cpp
        )

    set(incs include/pcl/${SUBSYS_NAME}/extract_clusters.h
        include/pcl/${SUBSYS_NAME}/extract_labeled_clusters.h
        include/pcl/${SUBSYS_NAME}/extract_polygonal_prism_data.h
        include/pcl/${SUBSYS_NAME}/sac_segmentation.h
        include/pcl/${SUBSYS_NAME}/seeded_hue_segmentation.h
        include/pcl/${SUBSYS_NAME}/segment_differences.h
        include/pcl/${SUBSYS_NAME}/region_growing.h
        )

    set(impl_incs include/pcl/${SUBSYS_NAME}/impl/extract_clusters.hpp
        include/pcl/${SUBSYS_NAME}/impl/extract_labeled_clusters.hpp
        include/pcl/${SUBSYS_NAME}/impl/extract_polygonal_prism_data.hpp
        include/pcl/${SUBSYS_NAME}/impl/sac_segmentation.hpp
        include/pcl/${SUBSYS_NAME}/impl/seeded_hue_segmentation.hpp
        include/pcl/${SUBSYS_NAME}/impl/segment_differences.hpp
        include/pcl/${SUBSYS_NAME}/impl/region_growing.hpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    target_link_libraries(${LIB_NAME} pcl_search pcl_sample_consensus)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
endif(build)


