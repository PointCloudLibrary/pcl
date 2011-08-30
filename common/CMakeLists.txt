set(SUBSYS_NAME common)
set(SUBSYS_DESC "Point cloud common library")
set(SUBSYS_DEPS)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(srcs src/pcl_base.cpp
        src/common.cpp
        src/distances.cpp
        src/intersections.cpp
        src/parse.cpp
        src/poses_from_matches.cpp
        src/print.cpp
        src/time_trigger.cpp
        )

    set(incs 
        include/pcl/exceptions.h
        include/pcl/pcl_base.h
        include/pcl/pcl_macros.h
        include/pcl/point_cloud.h
        include/pcl/point_representation.h
        include/pcl/point_types.h
        include/pcl/win32_macros.h
        )
        
    if(NOT USE_ROS)
        set(incs ${incs}
          include/pcl/ModelCoefficients.h
          include/pcl/PolygonMesh.h
          include/pcl/Vertices.h
          include/pcl/PointIndices.h
          include/pcl/TextureMesh.h
        )

        set(sensor_msg_incs 
          include/sensor_msgs/PointField.h
          include/sensor_msgs/PointCloud2.h
          include/sensor_msgs/Image.h
          )

        set(std_msg_incs include/std_msgs/Header.h)
    endif(NOT USE_ROS)
    
    set(common_incs include/pcl/common/angles.h
        include/pcl/common/bivariate_polynomial.h
        include/pcl/common/centroid.h
        include/pcl/common/concatenate.h
        include/pcl/common/common.h
        include/pcl/common/common_headers.h
        include/pcl/common/distances.h
        include/pcl/common/eigen.h
        include/pcl/common/file_io.h
        include/pcl/common/intersections.h
        include/pcl/common/norms.h
        include/pcl/common/piecewise_linear_function.h
        include/pcl/common/point_correspondence.h
        include/pcl/common/polynomial_calculations.h
        include/pcl/common/poses_from_matches.h
        include/pcl/common/rigid_transforms.h
        include/pcl/common/time.h
        include/pcl/common/time_trigger.h
        include/pcl/common/transform.h
        include/pcl/common/transformation_from_correspondences.h
        include/pcl/common/vector_average.h
        include/pcl/common/pca.h
        include/pcl/common/synchronizer.h
        )

    set(common_incs_impl
        include/pcl/common/impl/angles.hpp
        include/pcl/common/impl/bivariate_polynomial.hpp
        include/pcl/common/impl/centroid.hpp
        include/pcl/common/impl/common.hpp
        include/pcl/common/impl/file_io.hpp
        include/pcl/common/impl/norms.hpp
        include/pcl/common/impl/piecewise_linear_function.hpp
        include/pcl/common/impl/polynomial_calculations.hpp
        include/pcl/common/impl/rigid_transforms.hpp
        include/pcl/common/impl/pca.hpp
        include/pcl/common/impl/transform.hpp
        include/pcl/common/impl/transformation_from_correspondences.hpp
        include/pcl/common/impl/vector_average.hpp
       )

    set(impl_incs include/pcl/impl/instantiate.hpp
        include/pcl/impl/point_types.hpp
        )

    set(ros_incs include/pcl/ros/conversions.h
        include/pcl/ros/for_each_type.h
        include/pcl/ros/point_traits.h
        include/pcl/ros/register_point_struct.h
        )

    set(tools_incs 
        include/pcl/console/parse.h
        include/pcl/console/print.h
        include/pcl/console/time.h
        )


    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${common_incs} ${impl_incs} ${ros_incs} ${tools_incs} ${common_incs_impl} ${sensor_msg_incs} ${std_msg_incs})
    link_ros_libs (${LIB_NAME})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" ""
        "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} "" ${incs})
    if(NOT USE_ROS)
        PCL_ADD_INCLUDES(${SUBSYS_NAME} "../sensor_msgs" ${sensor_msg_incs})
        PCL_ADD_INCLUDES(${SUBSYS_NAME} "../std_msgs" ${std_msg_incs})
    endif(NOT USE_ROS)
    PCL_ADD_INCLUDES(${SUBSYS_NAME} common ${common_incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} common/impl ${common_incs_impl})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} impl ${impl_incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ros ${ros_incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} console ${tools_incs})

    add_subdirectory(test)
endif(build)
