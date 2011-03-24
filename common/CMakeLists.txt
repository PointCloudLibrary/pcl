set(SUBSYS_NAME common)
set(SUBSYS_DESC "Point cloud common library")
set(SUBSYS_DEPS)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON ${SUBSYS_DEPS})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(srcs src/pcl_base.cpp
        src/distances.cpp
        src/intersections.cpp
        src/parse.cpp
        src/poses_from_matches.cpp
        src/print.cpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "" ""
        "" "")

    # Install include files
    set(incs include/pcl/exceptions.h
        include/pcl/pcl_base.h
        include/pcl/pcl_macros.h
        include/pcl/point_cloud.h
        include/pcl/point_representation.h
        include/pcl/point_types.h
        include/pcl/ros_macros.h
        include/pcl/view.h
        include/pcl/win32_macros.h
        include/pcl/ModelCoefficients.h
        include/pcl/PolygonMesh.h
        include/pcl/Vertices.h
        include/pcl/PointIndices.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} "pcl" ${incs})
    set(sensor_msg_incs include/sensor_msgs/PointField.h
        include/sensor_msgs/PointCloud2.h
        include/sensor_msgs/CameraInfo.h
        include/sensor_msgs/image_encodings.h
        include/sensor_msgs/Image.h
        include/sensor_msgs/RegionOfInterest.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} sensor_msgs ${sensor_msg_incs})
    set(std_msg_incs include/std_msgs/Header.h)
    PCL_ADD_INCLUDES(${SUBSYS_NAME} std_msgs ${std_msg_incs})
    set(common_incs include/pcl/common/angles.h
        include/pcl/common/angles.hpp
        include/pcl/common/bivariate_polynomial.h
        include/pcl/common/bivariate_polynomial.hpp
        include/pcl/common/centroid.h
        include/pcl/common/common.h
        include/pcl/common/common.hpp
        include/pcl/common/common_headers.h
        include/pcl/common/distances.h
        include/pcl/common/eigen.h
        include/pcl/common/file_io.h
        include/pcl/common/file_io.hpp
        include/pcl/common/intersections.h
        include/pcl/common/norms.h
        include/pcl/common/norms.hpp
        include/pcl/common/piecewise_linear_function.h
        include/pcl/common/piecewise_linear_function.hpp
        include/pcl/common/point_correspondence.h
        include/pcl/common/polynomial_calculations.h
        include/pcl/common/polynomial_calculations.hpp
        include/pcl/common/poses_from_matches.h
        include/pcl/common/rigid_transformations.h
        include/pcl/common/time.h
        include/pcl/common/time.hpp
        include/pcl/common/transform.h
        include/pcl/common/transform.hpp
        include/pcl/common/transformation_from_correspondences.h
        include/pcl/common/transformation_from_correspondences.hpp
        include/pcl/common/vector_average.h
        include/pcl/common/vector_average.hpp
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} common ${common_incs})
    set(common_incs_impl
        include/pcl/common/impl/centroid.hpp
        include/pcl/common/impl/rigid_transformations.hpp
       )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} common/impl ${common_incs_impl})
    set(impl_incs include/pcl/impl/instantiate.hpp
        include/pcl/impl/point_types.hpp
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} impl ${impl_incs})
    set(ros_incs include/pcl/ros/conversions.h
        include/pcl/ros/for_each_type.h
        include/pcl/ros/point_traits.h
        include/pcl/ros/register_point_struct.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ros ${ros_incs})
    set(tools_incs include/pcl/terminal_tools/parse.h
        include/pcl/terminal_tools/print.h
        include/pcl/terminal_tools/time.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} tools ${tools_incs})

    add_subdirectory(test)
endif(build)

