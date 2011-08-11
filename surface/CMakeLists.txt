set(SUBSYS_NAME surface)
set(SUBSYS_DESC "Point cloud surface library")
set(SUBSYS_DEPS common io features kdtree registration)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    if(QHULL_FOUND)
        include_directories(${QHULL_INCLUDE_DIRS})
        set(HULL_INCLUDES
            include/pcl/${SUBSYS_NAME}/concave_hull.h
            include/pcl/${SUBSYS_NAME}/convex_hull.h
            )
        set(HULL_IMPLS
            include/pcl/${SUBSYS_NAME}/impl/concave_hull.hpp
            include/pcl/${SUBSYS_NAME}/impl/convex_hull.hpp
            )
        set(HULL_SOURCES
            src/concave_hull.cpp
            src/convex_hull.cpp
            )
    endif(QHULL_FOUND)
    
    if(VTK_FOUND)
        set(VTK_USE_FILE ${VTK_USE_FILE} CACHE INTERNAL "VTK_USE_FILE")
        include (${VTK_USE_FILE})
        set(VTK_SURFACE_INCLUDES include/pcl/${SUBSYS_NAME}/vtk_smoother.h)
        set(VTK_SURFACE_SOURCE src/vtk_smoother.cpp)
        set(VTK_SURFACE_TARGET_LINK_LIBRARIES vtkCommon vtkWidgets)
    endif(VTK_FOUND)

    set(srcs src/gp3.cpp
        src/grid_projection.cpp
        src/organized_fast_mesh.cpp
        src/mls.cpp
        src/simplification_remove_unused_vertices.cpp
        src/surfel_smoothing.cpp
        src/marching_cubes.cpp
        src/marching_cubes_greedy.cpp
        src/marching_cubes_greedy_dot.cpp
        src/texture_mapping.cpp
        ${VTK_SURFACE_SOURCE}
        ${HULL_SOURCES}
        )

    set(incs include/pcl/${SUBSYS_NAME}/gp3.h
        include/pcl/${SUBSYS_NAME}/grid_projection.h
        include/pcl/${SUBSYS_NAME}/mls.h
        include/pcl/${SUBSYS_NAME}/organized_fast_mesh.h
        include/pcl/${SUBSYS_NAME}/simplification_remove_unused_vertices.h
        include/pcl/${SUBSYS_NAME}/surfel_smoothing.h
        include/pcl/${SUBSYS_NAME}/reconstruction.h
        include/pcl/${SUBSYS_NAME}/marching_cubes.h
        include/pcl/${SUBSYS_NAME}/marching_cubes_greedy.h
        include/pcl/${SUBSYS_NAME}/marching_cubes_greedy_dot.h
        include/pcl/${SUBSYS_NAME}/texture_mapping.h
        ${VTK_SURFACE_INCLUDES}
        ${HULL_INCLUDES}
        )

    set(impl_incs include/pcl/${SUBSYS_NAME}/impl/gp3.hpp
        include/pcl/${SUBSYS_NAME}/impl/grid_projection.hpp
        include/pcl/${SUBSYS_NAME}/impl/mls.hpp
        include/pcl/${SUBSYS_NAME}/impl/organized_fast_mesh.hpp
        include/pcl/${SUBSYS_NAME}/impl/surfel_smoothing.hpp
        include/pcl/${SUBSYS_NAME}/impl/reconstruction.hpp
        include/pcl/${SUBSYS_NAME}/impl/marching_cubes.hpp
        include/pcl/${SUBSYS_NAME}/impl/marching_cubes_greedy.hpp
        include/pcl/${SUBSYS_NAME}/impl/marching_cubes_greedy_dot.hpp
        include/pcl/${SUBSYS_NAME}/impl/texture_mapping.hpp
        ${HULL_IMPLS}
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${VTK_INCLUDE_DIRS})
    link_directories(${VTK_LIBRARY_DIRS})
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_common pcl_io pcl_kdtree ${OpenCV_LIBS} ${VTK_SURFACE_TARGET_LINK_LIBRARIES} ${QHULL_LIBRARIES})
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    add_subdirectory(test)
endif(build)
