set(SUBSYS_NAME filters)
set(SUBSYS_DESC "Point cloud filters library")
set(SUBSYS_DEPS common sample_consensus io kdtree)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(srcs src/conditional_removal.cpp
        src/extract_indices.cpp
        src/filter.cpp
        src/passthrough.cpp
        src/project_inliers.cpp
        src/radius_outlier_removal.cpp
        src/statistical_outlier_removal.cpp
        src/voxel_grid.cpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_common pcl_sample_consensus
        pcl_kdtree ${Boost_LIBRARIES})
    PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
        "${EXT_DEPS}" "" "" "")

    # Install include files
    set(incs include/pcl/${SUBSYS_NAME}/conditional_removal.h
        include/pcl/${SUBSYS_NAME}/extract_indices.h
        include/pcl/${SUBSYS_NAME}/filter.h
        include/pcl/${SUBSYS_NAME}/passthrough.h
        include/pcl/${SUBSYS_NAME}/project_inliers.h
        include/pcl/${SUBSYS_NAME}/radius_outlier_removal.h
        include/pcl/${SUBSYS_NAME}/statistical_outlier_removal.h
        include/pcl/${SUBSYS_NAME}/voxel_grid.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})

    set(impl_incs include/pcl/${SUBSYS_NAME}/impl/conditional_removal.hpp
        include/pcl/${SUBSYS_NAME}/impl/extract_indices.hpp
        include/pcl/${SUBSYS_NAME}/impl/filter.hpp
        include/pcl/${SUBSYS_NAME}/impl/passthrough.hpp
        include/pcl/${SUBSYS_NAME}/impl/project_inliers.hpp
        include/pcl/${SUBSYS_NAME}/impl/radius_outlier_removal.hpp
        include/pcl/${SUBSYS_NAME}/impl/statistical_outlier_removal.hpp
        include/pcl/${SUBSYS_NAME}/impl/voxel_grid.hpp
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
endif(build)


