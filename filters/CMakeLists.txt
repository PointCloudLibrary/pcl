set(SUBSYS_NAME filters)
set(SUBSYS_DESC "Point cloud filters library")
set(SUBSYS_DEPS common sample_consensus io kdtree)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(srcs 
        src/conditional_removal.cpp
        src/extract_indices.cpp
        src/filter.cpp
        src/passthrough.cpp
        src/project_inliers.cpp
        src/radius_outlier_removal.cpp
        src/statistical_outlier_removal.cpp
        src/voxel_grid.cpp
        )

    set(incs 
        include/pcl/${SUBSYS_NAME}/conditional_removal.h
        include/pcl/${SUBSYS_NAME}/extract_indices.h
        include/pcl/${SUBSYS_NAME}/filter.h
        include/pcl/${SUBSYS_NAME}/passthrough.h
        include/pcl/${SUBSYS_NAME}/project_inliers.h
        include/pcl/${SUBSYS_NAME}/radius_outlier_removal.h
        include/pcl/${SUBSYS_NAME}/statistical_outlier_removal.h
        include/pcl/${SUBSYS_NAME}/voxel_grid.h
        )

    set(impl_incs 
        include/pcl/${SUBSYS_NAME}/impl/conditional_removal.hpp
        include/pcl/${SUBSYS_NAME}/impl/extract_indices.hpp
        include/pcl/${SUBSYS_NAME}/impl/filter.hpp
        include/pcl/${SUBSYS_NAME}/impl/passthrough.hpp
        include/pcl/${SUBSYS_NAME}/impl/project_inliers.hpp
        include/pcl/${SUBSYS_NAME}/impl/radius_outlier_removal.hpp
        include/pcl/${SUBSYS_NAME}/impl/statistical_outlier_removal.hpp
        include/pcl/${SUBSYS_NAME}/impl/voxel_grid.hpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_common pcl_sample_consensus pcl_kdtree)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    add_subdirectory(test)
    add_subdirectory(tools)
endif(build)
