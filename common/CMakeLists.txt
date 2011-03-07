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
    set(incs include/exceptions.h
        include/pcl_base.h
        include/pcl_macros.h
        include/point_cloud.h
        include/point_representation.h
        include/point_types.h
        include/view.h
        include/win32_macros.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} "" ${incs})
    set(common_incs include/common/angles.h
        include/common/angles.hpp
        include/common/bivariate_polynomial.h
        include/common/bivariate_polynomial.hpp
        include/common/common.h
        include/common/common.hpp
        include/common/common_headers.h
        include/common/distances.h
        include/common/eigen.h
        include/common/file_io.h
        include/common/file_io.hpp
        include/common/intersections.h
        include/common/norms.h
        include/common/norms.hpp
        include/common/piecewise_linear_function.h
        include/common/piecewise_linear_function.hpp
        include/common/point_correspondence.h
        include/common/polynomial_calculations.h
        include/common/polynomial_calculations.hpp
        include/common/poses_from_matches.h
        include/common/time.h
        include/common/time.hpp
        include/common/transform.h
        include/common/transform.hpp
        include/common/transformation_from_correspondences.h
        include/common/transformation_from_correspondences.hpp
        include/common/vector_average.h
        include/common/vector_average.hpp
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} common ${common_incs})
endif(build)

