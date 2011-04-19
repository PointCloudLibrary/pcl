set(SUBSYS_NAME surface)
set(SUBSYS_DESC "Point cloud surface library")
set(SUBSYS_DEPS common io features kdtree registration)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    find_package(Qhull)
    if(QHULL_FOUND)
        include_directories(${QHULL_INCLUDE_DIRS})
        set(HULL_INCLUDES
            include/pcl/surface/concave_hull.h
            include/pcl/surface/convexe_hull.h
            )
        set(HULL_IMPLS
            include/pcl/surfaces/impl/concave_hull.hpp
            include/pcl/surfaces/impl/convex_hull.hpp
            )
        set(HULL_SOURCES
            src/concave_hull.cpp
            src/convex_hull.cpp
            )
    endif(QHULL_FOUND)

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

    set(srcs src/gp3.cpp
        src/grid_projection.cpp
        src/mls.cpp
        ${HULL_SOURCES}
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} ${Boost_LIBRARIES})
    if(QHULL_FOUND)
        target_link_libraries(${LIB_NAME} ${QHULL_LIBRARIES})
    endif(QHULL_FOUND)
    PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
        "${EXT_DEPS}" "" "" "")

    # Install include files
    set(incs include/pcl/${SUBSYS_NAME}/gp3.h
        include/pcl/${SUBSYS_NAME}/grid_projection.h
        include/pcl/${SUBSYS_NAME}/mls.h
        include/pcl/${SUBSYS_NAME}/reconstruction.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs} ${HULL_INCLUDES})

    set(impl_incs include/pcl/${SUBSYS_NAME}/impl/gp3.hpp
        include/pcl/${SUBSYS_NAME}/impl/grid_projection.hpp
        include/pcl/${SUBSYS_NAME}/impl/mls.hpp
        include/pcl/${SUBSYS_NAME}/impl/reconstruction.hpp
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${impl_incs} ${HULL_IMPLS})

    add_subdirectory(test)
endif(build)
