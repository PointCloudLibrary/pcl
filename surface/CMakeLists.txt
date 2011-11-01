set(SUBSYS_NAME surface)
set(SUBSYS_DESC "Point cloud surface library")
set(SUBSYS_DEPS common search kdtree octree)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS qhull)

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

    if (VTK_FOUND AND NOT ANDROID_NDK)
        set(VTK_USE_FILE ${VTK_USE_FILE} CACHE INTERNAL "VTK_USE_FILE")
        include (${VTK_USE_FILE})
        set(VTK_SURFACE_INCLUDES include/pcl/${SUBSYS_NAME}/vtk_smoother.h)
        set(VTK_SURFACE_SOURCE src/vtk_smoother.cpp)
        set(VTK_SURFACE_TARGET_LINK_LIBRARIES vtkCommon vtkWidgets)
    endif()
    
    ####### NURBS FITTING by thomas.moerwald #########

    #SET(BUILD_surface_nurbs 0 CACHE BOOL "Fitting NURBS to point-clouds" )
    #IF(BUILD_surface_nurbs)
    
		# TODO remove this ugly thing
    #set(OPENNURBS_PATH /home/tm/svn-repositories/v4r/trunk/external/opennurbs)
    #link_directories(${OPENNURBS_PATH})
    #include_directories(${OPENNURBS_PATH})
    #set(NURBS_INCLUDES
    #		include/pcl/${SUBSYS_NAME}/nurbs/NurbsData.h
    #		include/pcl/${SUBSYS_NAME}/nurbs/NurbsFitting.h
    #	include/pcl/${SUBSYS_NAME}/nurbs/NurbsTools.h
    #		)
    #set(NURBS_SOURCES
    #		src/nurbs/NurbsFitting.cpp
    #		src/nurbs/NurbsTools.cpp
    #		)
    #ENDIF(BUILD_surface_nurbs)
    

    #    # Includes for Misha's Poisson reconstruction code
    #    set(POISSON_INCLUDES
    #	include/pcl/${SUBSYS_NAME}/poisson/Allocator.h
    #	include/pcl/${SUBSYS_NAME}/poisson/BinaryNode.h
    #	include/pcl/${SUBSYS_NAME}/poisson/CmdLineParser.h
    #	include/pcl/${SUBSYS_NAME}/poisson/Factor.h
    #	include/pcl/${SUBSYS_NAME}/poisson/FunctionData.h
    #	include/pcl/${SUBSYS_NAME}/poisson/Geometry.h
    #	include/pcl/${SUBSYS_NAME}/poisson/Hash.h
    #	include/pcl/${SUBSYS_NAME}/poisson/MarchingCubes.h
    #	include/pcl/${SUBSYS_NAME}/poisson/MemoryUsage.h
    #	include/pcl/${SUBSYS_NAME}/poisson/MultiGridOctreeData.h
    #	include/pcl/${SUBSYS_NAME}/poisson/Octree.h
    #	include/pcl/${SUBSYS_NAME}/poisson/Polynomial.h
    #	include/pcl/${SUBSYS_NAME}/poisson/PPolynomial.h
    #	include/pcl/${SUBSYS_NAME}/poisson/SparseMatrix.h
    #	include/pcl/${SUBSYS_NAME}/poisson/Time.h
    #	include/pcl/${SUBSYS_NAME}/poisson/Vector.h
    #	)
    #    set(POISSON_IMPLS
    #        include/pcl/${SUBSYS_NAME}/impl/poisson/FunctionData.hpp
    #        include/pcl/${SUBSYS_NAME}/impl/poisson/Geometry.hpp
    #        include/pcl/${SUBSYS_NAME}/impl/poisson/MultiGridOctreeData.hpp
    #        include/pcl/${SUBSYS_NAME}/impl/poisson/Octree.hpp
    #        include/pcl/${SUBSYS_NAME}/impl/poisson/Polynomial.hpp
    #        include/pcl/${SUBSYS_NAME}/impl/poisson/PPolynomial.hpp
    #        include/pcl/${SUBSYS_NAME}/impl/poisson/SparseMatrix.hpp
    #        include/pcl/${SUBSYS_NAME}/impl/poisson/Vector.hpp
    #	)
    #    set(POISSON_SOURCES
    #	src/poisson/CmdLineParser.cpp
    #	src/poisson/Factor.cpp
    #	src/poisson/Geometry.cpp
    #	src/poisson/MarchingCubes.cpp
    #	src/poisson/Time.cpp
    #	)
    #
    set(srcs src/ear_clipping.cpp
        src/gp3.cpp
        src/grid_projection.cpp
        src/marching_cubes.cpp
        src/marching_cubes_greedy.cpp
        src/marching_cubes_greedy_dot.cpp
        src/mls.cpp
        src/organized_fast_mesh.cpp
        src/simplification_remove_unused_vertices.cpp
        src/surfel_smoothing.cpp
        src/texture_mapping.cpp
        ${VTK_SURFACE_SOURCE}
        #src/poisson.cpp
        ${HULL_SOURCES}
        #${POISSON_SOURCES}
        )

    set(incs include/pcl/${SUBSYS_NAME}/ear_clipping.h
        include/pcl/${SUBSYS_NAME}/gp3.h
        include/pcl/${SUBSYS_NAME}/grid_projection.h
        include/pcl/${SUBSYS_NAME}/marching_cubes.h
        include/pcl/${SUBSYS_NAME}/marching_cubes_greedy.h
        include/pcl/${SUBSYS_NAME}/marching_cubes_greedy_dot.h
        include/pcl/${SUBSYS_NAME}/mls.h
        include/pcl/${SUBSYS_NAME}/organized_fast_mesh.h
        include/pcl/${SUBSYS_NAME}/reconstruction.h
        include/pcl/${SUBSYS_NAME}/simplification_remove_unused_vertices.h
        include/pcl/${SUBSYS_NAME}/surfel_smoothing.h
        include/pcl/${SUBSYS_NAME}/texture_mapping.h
        ${VTK_SURFACE_INCLUDES}
        #include/pcl/${SUBSYS_NAME}/poisson.h
        ${HULL_INCLUDES}
        #${POISSON_INCLUDES}
        )

    set(impl_incs include/pcl/${SUBSYS_NAME}/impl/gp3.hpp
        include/pcl/${SUBSYS_NAME}/impl/grid_projection.hpp
        include/pcl/${SUBSYS_NAME}/impl/marching_cubes.hpp
        include/pcl/${SUBSYS_NAME}/impl/marching_cubes_greedy.hpp
        include/pcl/${SUBSYS_NAME}/impl/marching_cubes_greedy_dot.hpp
        include/pcl/${SUBSYS_NAME}/impl/mls.hpp
        include/pcl/${SUBSYS_NAME}/impl/organized_fast_mesh.hpp
        include/pcl/${SUBSYS_NAME}/impl/reconstruction.hpp
        include/pcl/${SUBSYS_NAME}/impl/surfel_smoothing.hpp
        include/pcl/${SUBSYS_NAME}/impl/texture_mapping.hpp
        #include/pcl/${SUBSYS_NAME}/impl/poisson.hpp
        ${HULL_IMPLS}
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${VTK_INCLUDE_DIRS})
    link_directories(${VTK_LIBRARY_DIRS})
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_common pcl_io pcl_search pcl_kdtree pcl_octree ${OpenCV_LIBS} ${VTK_SURFACE_TARGET_LINK_LIBRARIES})
    if(QHULL_FOUND)
      target_link_libraries(${LIB_NAME} ${QHULL_LIBRARIES})
    endif(QHULL_FOUND)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    #PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/poisson ${POISSON_INCLUDES})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
    #PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl/poisson ${POISSON_IMPLS})

    add_subdirectory(test)
endif(build)
