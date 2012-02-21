set(SUBSYS_NAME filters)
set(SUBSYS_DESC "Point cloud filters library")
set(SUBSYS_DEPS common sample_consensus search kdtree octree)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC(${SUBSYS_NAME})

if(build)
    set(srcs
        src/conditional_removal.cpp
        src/crop_box.cpp
        src/extract_indices.cpp
        src/filter.cpp
        src/filter_indices.cpp
        src/passthrough.cpp
        src/project_inliers.cpp
        src/radius_outlier_removal.cpp
        src/random_sample.cpp
        src/normal_space.cpp
        src/statistical_outlier_removal.cpp
        src/voxel_grid.cpp
        src/approximate_voxel_grid.cpp
        src/bilateral.cpp
        src/crop_hull.cpp
	src/voxel_grid_covariance.cpp
        )

    set(incs
        include/pcl/${SUBSYS_NAME}/conditional_removal.h
        include/pcl/${SUBSYS_NAME}/crop_box.h
        include/pcl/${SUBSYS_NAME}/clipper3D.h
        include/pcl/${SUBSYS_NAME}/plane_clipper3D.h
        include/pcl/${SUBSYS_NAME}/box_clipper3D.h
        include/pcl/${SUBSYS_NAME}/crop_hull.h
        include/pcl/${SUBSYS_NAME}/extract_indices.h
        include/pcl/${SUBSYS_NAME}/filter.h
        include/pcl/${SUBSYS_NAME}/filter_indices.h
        include/pcl/${SUBSYS_NAME}/passthrough.h
        include/pcl/${SUBSYS_NAME}/project_inliers.h
        include/pcl/${SUBSYS_NAME}/radius_outlier_removal.h
        include/pcl/${SUBSYS_NAME}/random_sample.h
        include/pcl/${SUBSYS_NAME}/normal_space.h
        include/pcl/${SUBSYS_NAME}/statistical_outlier_removal.h
        include/pcl/${SUBSYS_NAME}/voxel_grid.h
        include/pcl/${SUBSYS_NAME}/approximate_voxel_grid.h
        include/pcl/${SUBSYS_NAME}/bilateral.h
	include/pcl/${SUBSYS_NAME}/voxel_grid_covariance.h
        )

    set(impl_incs
        include/pcl/${SUBSYS_NAME}/impl/conditional_removal.hpp
        include/pcl/${SUBSYS_NAME}/impl/crop_box.hpp
        include/pcl/${SUBSYS_NAME}/impl/crop_hull.hpp
        include/pcl/${SUBSYS_NAME}/impl/plane_clipper3D.hpp
        include/pcl/${SUBSYS_NAME}/impl/box_clipper3D.hpp
        include/pcl/${SUBSYS_NAME}/impl/extract_indices.hpp
        include/pcl/${SUBSYS_NAME}/impl/filter.hpp
        include/pcl/${SUBSYS_NAME}/impl/filter_indices.hpp
        include/pcl/${SUBSYS_NAME}/impl/passthrough.hpp
        include/pcl/${SUBSYS_NAME}/impl/project_inliers.hpp
        include/pcl/${SUBSYS_NAME}/impl/radius_outlier_removal.hpp
        include/pcl/${SUBSYS_NAME}/impl/random_sample.hpp
        include/pcl/${SUBSYS_NAME}/impl/normal_space.hpp
        include/pcl/${SUBSYS_NAME}/impl/statistical_outlier_removal.hpp
        include/pcl/${SUBSYS_NAME}/impl/voxel_grid.hpp
        include/pcl/${SUBSYS_NAME}/impl/approximate_voxel_grid.hpp
        include/pcl/${SUBSYS_NAME}/impl/bilateral.hpp
	include/pcl/${SUBSYS_NAME}/impl/voxel_grid_covariance.hpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    target_link_libraries(${LIB_NAME} pcl_common pcl_sample_consensus pcl_search pcl_kdtree pcl_octree)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    if(BUILD_TESTS)
        add_subdirectory(test)
    endif(BUILD_TESTS)

endif(build)
