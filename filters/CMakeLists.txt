set(SUBSYS_NAME filters)
set(SUBSYS_DESC "Point cloud filters library")
set(SUBSYS_DEPS common sample_consensus search kdtree octree)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

if(build)
    set(srcs
        src/conditional_removal.cpp
        src/crop_box.cpp
        src/extract_indices.cpp
        src/filter.cpp
        src/filter_indices.cpp
        src/passthrough.cpp
        src/color.cpp
        src/project_inliers.cpp
        src/radius_outlier_removal.cpp
        src/random_sample.cpp
        src/normal_space.cpp
        src/statistical_outlier_removal.cpp
        src/voxel_grid.cpp
        src/approximate_voxel_grid.cpp
        src/bilateral.cpp
        )

    set(incs
        include/pcl/${SUBSYS_NAME}/conditional_removal.h
        include/pcl/${SUBSYS_NAME}/crop_box.h
        include/pcl/${SUBSYS_NAME}/extract_indices.h
        include/pcl/${SUBSYS_NAME}/filter.h
        include/pcl/${SUBSYS_NAME}/filter_indices.h
        include/pcl/${SUBSYS_NAME}/passthrough.h
        include/pcl/${SUBSYS_NAME}/color.h
        include/pcl/${SUBSYS_NAME}/project_inliers.h
        include/pcl/${SUBSYS_NAME}/radius_outlier_removal.h
        include/pcl/${SUBSYS_NAME}/random_sample.h
        include/pcl/${SUBSYS_NAME}/normal_space.h
        include/pcl/${SUBSYS_NAME}/statistical_outlier_removal.h
        include/pcl/${SUBSYS_NAME}/voxel_grid.h
        include/pcl/${SUBSYS_NAME}/approximate_voxel_grid.h
        include/pcl/${SUBSYS_NAME}/bilateral.h
        )

    set(impl_incs
        include/pcl/${SUBSYS_NAME}/impl/conditional_removal.hpp
        include/pcl/${SUBSYS_NAME}/impl/crop_box.hpp
        include/pcl/${SUBSYS_NAME}/impl/extract_indices.hpp
        include/pcl/${SUBSYS_NAME}/impl/filter.hpp
        include/pcl/${SUBSYS_NAME}/impl/filter_indices.hpp
        include/pcl/${SUBSYS_NAME}/impl/passthrough.hpp
        include/pcl/${SUBSYS_NAME}/impl/color.hpp
        include/pcl/${SUBSYS_NAME}/impl/project_inliers.hpp
        include/pcl/${SUBSYS_NAME}/impl/radius_outlier_removal.hpp
        include/pcl/${SUBSYS_NAME}/impl/random_sample.hpp
        include/pcl/${SUBSYS_NAME}/impl/normal_space.hpp
        include/pcl/${SUBSYS_NAME}/impl/statistical_outlier_removal.hpp
        include/pcl/${SUBSYS_NAME}/impl/voxel_grid.hpp
        include/pcl/${SUBSYS_NAME}/impl/approximate_voxel_grid.hpp
        include/pcl/${SUBSYS_NAME}/impl/bilateral.hpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_common pcl_sample_consensus pcl_search pcl_kdtree pcl_octree)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    add_subdirectory(test)
endif(build)
