set(SUBSYS_NAME features)
set(SUBSYS_DESC "Point cloud features library")
set(SUBSYS_DEPS common io kdtree range_image)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

    set(srcs src/feature.cpp
        src/fpfh.cpp
        src/fpfh_omp.cpp
        src/integral_image_normal.cpp
        src/intensity_gradient.cpp
        src/intensity_spin.cpp
        src/moment_invariants.cpp
        src/narf.cpp
        src/narf_descriptor.cpp
        src/normal_3d.cpp
        src/normal_3d_omp.cpp
        src/normal_3d_tbb.cpp
        src/pfh.cpp
        src/principal_curvatures.cpp
        src/rift.cpp
        src/rsd.cpp
        src/vfh.cpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    PCL_ADD_OPENMP_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_range_image ${Boost_LIBRARIES})
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
        "${SUBSYS_DEPS}" "" "" "")

    set(srcs src/boundary.cpp
        src/range_image_border_extractor.cpp
        )

    set(LIB_NAME pcl_range_image_border_extractor)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs})
    PCL_ADD_OPENMP_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_range_image)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
        "${SUBSYS_DEPS}" "" "" "")

    # Install include files
    set(incs include/pcl/${SUBSYS_NAME}/boundary.h
        include/pcl/${SUBSYS_NAME}/feature.h
        include/pcl/${SUBSYS_NAME}/fpfh.h
        include/pcl/${SUBSYS_NAME}/fpfh_omp.h
        include/pcl/${SUBSYS_NAME}/integral_image_2d.h
        include/pcl/${SUBSYS_NAME}/integral_image_normal.h
        include/pcl/${SUBSYS_NAME}/intensity_gradient.h
        include/pcl/${SUBSYS_NAME}/intensity_spin.h
        include/pcl/${SUBSYS_NAME}/moment_invariants.h
        include/pcl/${SUBSYS_NAME}/narf.h
        include/pcl/${SUBSYS_NAME}/narf_descriptor.h
        include/pcl/${SUBSYS_NAME}/normal_3d.h
        include/pcl/${SUBSYS_NAME}/normal_3d_omp.h
        include/pcl/${SUBSYS_NAME}/normal_3d_tbb.h
        include/pcl/${SUBSYS_NAME}/pfh.h
        include/pcl/${SUBSYS_NAME}/principal_curvature.h
        include/pcl/${SUBSYS_NAME}/range_image_border_extractor.h
        include/pcl/${SUBSYS_NAME}/rift.h
        include/pcl/${SUBSYS_NAME}/rsd.h
        include/pcl/${SUBSYS_NAME}/vfh.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    set(impl_incs include/pcl/${SUBSYS_NAME}/boundary.hpp
        include/pcl/${SUBSYS_NAME}/feature.hpp
        include/pcl/${SUBSYS_NAME}/fpfh.hpp
        include/pcl/${SUBSYS_NAME}/fpfh_omp.hpp
        include/pcl/${SUBSYS_NAME}/integral_image_2d.hpp
        include/pcl/${SUBSYS_NAME}/intensity_gradient.hpp
        include/pcl/${SUBSYS_NAME}/intensity_spin.hpp
        include/pcl/${SUBSYS_NAME}/moment_invariants.hpp
        include/pcl/${SUBSYS_NAME}/narf.hpp
        include/pcl/${SUBSYS_NAME}/normal_3d.hpp
        include/pcl/${SUBSYS_NAME}/normal_3d_omp.hpp
        include/pcl/${SUBSYS_NAME}/normal_3d_tbb.hpp
        include/pcl/${SUBSYS_NAME}/pfh.hpp
        include/pcl/${SUBSYS_NAME}/principal_curvature.hpp
        include/pcl/${SUBSYS_NAME}/range_image_border_extractor.hpp
        include/pcl/${SUBSYS_NAME}/rift.hpp
        include/pcl/${SUBSYS_NAME}/rsd.hpp
        include/pcl/${SUBSYS_NAME}/vfh.hpp
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
endif(build)

