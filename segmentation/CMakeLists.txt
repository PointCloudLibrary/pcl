set(SUBSYS_NAME segmentation)
set(SUBSYS_DESC "Point cloud segmentation library")
set(SUBSYS_DEPS common kdtree sample_consensus)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(srcs src/extract_clusters.cpp
        src/extract_polygonal_prism_data.cpp
        src/sac_segmentation.cpp
        src/segment_differences.cpp
        )

    set(incs include/pcl/${SUBSYS_NAME}/extract_clusters.h
        include/pcl/${SUBSYS_NAME}/extract_polygonal_prism_data.h
        include/pcl/${SUBSYS_NAME}/sac_segmentation.h
        include/pcl/${SUBSYS_NAME}/segment_differences.h
        )

    set(impl_incs include/pcl/${SUBSYS_NAME}/impl/extract_clusters.hpp
        include/pcl/${SUBSYS_NAME}/impl/extract_polygonal_prism_data.hpp
        include/pcl/${SUBSYS_NAME}/impl/sac_segmentation.hpp
        include/pcl/${SUBSYS_NAME}/impl/segment_differences.hpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_kdtree pcl_sample_consensus)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
endif(build)


