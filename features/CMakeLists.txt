set(SUBSYS_NAME features)
set(SUBSYS_DESC "Point cloud features library")
set(SUBSYS_DEPS common kdtree range_image)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

    set(incs include/pcl/${SUBSYS_NAME}/cvfh.h
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
        include/pcl/${SUBSYS_NAME}/pfh.h
        include/pcl/${SUBSYS_NAME}/principal_curvatures.h
        include/pcl/${SUBSYS_NAME}/rift.h
        include/pcl/${SUBSYS_NAME}/rsd.h
        include/pcl/${SUBSYS_NAME}/vfh.h
        )

    set(impl_incs include/pcl/${SUBSYS_NAME}/impl/cvfh.hpp
        include/pcl/${SUBSYS_NAME}/impl/feature.hpp
        include/pcl/${SUBSYS_NAME}/impl/fpfh.hpp
        include/pcl/${SUBSYS_NAME}/impl/fpfh_omp.hpp
        include/pcl/${SUBSYS_NAME}/impl/integral_image_2d.hpp
        include/pcl/${SUBSYS_NAME}/impl/integral_image_normal.hpp
        include/pcl/${SUBSYS_NAME}/impl/intensity_gradient.hpp
        include/pcl/${SUBSYS_NAME}/impl/intensity_spin.hpp
        include/pcl/${SUBSYS_NAME}/impl/moment_invariants.hpp
        include/pcl/${SUBSYS_NAME}/impl/narf.hpp
        include/pcl/${SUBSYS_NAME}/impl/normal_3d.hpp
        include/pcl/${SUBSYS_NAME}/impl/normal_3d_omp.hpp
        include/pcl/${SUBSYS_NAME}/impl/pfh.hpp
        include/pcl/${SUBSYS_NAME}/impl/principal_curvatures.hpp
        include/pcl/${SUBSYS_NAME}/impl/rift.hpp
        include/pcl/${SUBSYS_NAME}/impl/rsd.hpp
        include/pcl/${SUBSYS_NAME}/impl/vfh.hpp
        )

    set(srcs src/feature.cpp
        src/cvfh.cpp
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
        src/pfh.cpp
        src/principal_curvatures.cpp
        src/rift.cpp
        src/rsd.cpp
        src/vfh.cpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    PCL_ADD_OPENMP_FLAGS(${LIB_NAME})
    PCL_LINK_OPENMP(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_range_image pcl_kdtree)
    PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
        "${EXT_DEPS}" "" "" "")
    # Install headers
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    set(border_srcs src/boundary.cpp
        src/range_image_border_extractor.cpp
        )

    set(border_incs include/pcl/${SUBSYS_NAME}/boundary.h
        include/pcl/${SUBSYS_NAME}/range_image_border_extractor.h
        )

    set(border_impl_incs include/pcl/${SUBSYS_NAME}/impl/boundary.hpp
        include/pcl/${SUBSYS_NAME}/impl/range_image_border_extractor.hpp
        )

    set(LIB_NAME pcl_range_image_border_extractor)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${border_srcs} ${border_incs} ${border_impl_incs})
    PCL_ADD_OPENMP_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_range_image)
    PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME}
        "Point cloud range image border extractor library"
        "${EXT_DEPS}" "" "" "")
    # Install headers
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${border_incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${border_impl_incs})
    
    add_subdirectory(test)
endif(build)
