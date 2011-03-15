set(SUBSYS_NAME keypoints)
set(SUBSYS_DESC "Point cloud keypoints library")
set(SUBSYS_DEPS common features io kdtree range_image)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(srcs src/narf_keypoint.cpp)

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    PCL_ADD_OPENMP_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_range_image_border_extractor)
    PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
    list(APPEND EXT_DEPS pcl_range_image_border_extractor)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
        "${EXT_DEPS}" "" "" "")

    # Install include files
    set(incs include/pcl/${SUBSYS_NAME}/keypoint.h
        include/pcl/${SUBSYS_NAME}/narf_keypoint.h
        include/pcl/${SUBSYS_NAME}/sift_keypoint.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})

    set(impl_incs include/pcl/${SUBSYS_NAME}/impl/keypoint.hpp
        include/pcl/${SUBSYS_NAME}/impl/sift_keypoint.hpp
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    add_subdirectory(test)
endif(build)

