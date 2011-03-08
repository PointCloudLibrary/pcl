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
        src/poses_from_matches.cpp
        )

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(pcl_common ${SUBSYS_NAME} ${srcs})
    PCL_ADD_CFLAGS(pcl_common ${SSE_FLAGS})
    PCL_MAKE_PKGCONFIG(pcl_common ${SUBSYS_NAME} "${SUBSYS_DESC}" "" ""
        "" "")

    # Install include files
    set(incs include/pcl/point_representation.h
        include/pcl/point_types.h
        include/pcl/win32_macros.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} "" ${incs})
    set(impl_incs include/common/pcl/impl/instantiate.hpp
        include/common/pcl/impl/point_types.hpp
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} impl ${impl_incs})
    set(ros_incs include/common/ros/conversions.h
        include/common/ros/for_each_type.h
        include/common/ros/point_traits.h
        include/common/ros/register_point_struct.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ros ${ros_incs})
endif(build)

