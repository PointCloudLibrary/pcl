set(SUBSYS_NAME surface)
set(SUBSYS_DESC "Point cloud surface library")
set(SUBSYS_DEPS common search kdtree octree)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} OPT_DEPS qhull)

PCL_ADD_DOC(${SUBSYS_NAME})

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
        
        set(VTK_SMOOTHING_INCLUDES include/pcl/${SUBSYS_NAME}/vtk_smoothing/vtk_utils.h
                                   include/pcl/${SUBSYS_NAME}/vtk_smoothing/vtk_mesh_subdivision.h
                                   include/pcl/${SUBSYS_NAME}/vtk_smoothing/vtk_mesh_smoothing_laplacian.h
                                   include/pcl/${SUBSYS_NAME}/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.h)
        set(VTK_SMOOTHING_SOURCE src/vtk_smoothing/vtk_utils.cpp
                                 src/vtk_smoothing/vtk_mesh_subdivision.cpp
                                 src/vtk_smoothing/vtk_mesh_smoothing_laplacian.cpp
                                 src/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.cpp)
        set(VTK_SMOOTHING_TARGET_LINK_LIBRARIES vtkCommon vtkWidgets)
    endif()
    
    ####### NURBS FITTING by thomas.moerwald #########
    SET(BUILD_surface_nurbs 0 CACHE BOOL "Fitting NURBS to point-clouds" )
    IF(BUILD_surface_nurbs)
    set(NURBS_INCLUDES
		include/pcl/${SUBSYS_NAME}/nurbs/eigen_defs.h
		include/pcl/${SUBSYS_NAME}/nurbs/nurbs_basis.h
		include/pcl/${SUBSYS_NAME}/nurbs/nurbs_curve.h
		include/pcl/${SUBSYS_NAME}/nurbs/nurbs_surface.h
    		include/pcl/${SUBSYS_NAME}/nurbs/nurbs_data.h
    		include/pcl/${SUBSYS_NAME}/nurbs/nurbs_fitting.h
    		include/pcl/${SUBSYS_NAME}/nurbs/nurbs_tools.h
    		include/pcl/${SUBSYS_NAME}/nurbs/nurbs_fitter.h
    		)
    set(NURBS_SOURCES
		src/nurbs/nurbs_basis.cpp
		src/nurbs/nurbs_curve.cpp
		src/nurbs/nurbs_surface.cpp
    		src/nurbs/nurbs_fitting.cpp
    		src/nurbs/nurbs_tools.cpp
    		src/nurbs/nurbs_fitter.cpp
    		)
    set(NURBS_IMPL
    		include/pcl/${SUBSYS_NAME}/impl/nurbs/nurbs_fitter.hpp
    		)
    ENDIF(BUILD_surface_nurbs)
    

    set(POISSON_INCLUDES
    	include/pcl/${SUBSYS_NAME}/poisson/allocator.h
    	include/pcl/${SUBSYS_NAME}/poisson/binary_node.h
    	include/pcl/${SUBSYS_NAME}/poisson/cmd_line_parser.h
    	include/pcl/${SUBSYS_NAME}/poisson/factor.h
    	include/pcl/${SUBSYS_NAME}/poisson/function_data.h
    	include/pcl/${SUBSYS_NAME}/poisson/geometry.h
    	include/pcl/${SUBSYS_NAME}/poisson/hash.h
    	include/pcl/${SUBSYS_NAME}/poisson/marching_cubes.h
    	include/pcl/${SUBSYS_NAME}/poisson/memory_usage.h
    	include/pcl/${SUBSYS_NAME}/poisson/multi_grid_octree_data.h
    	include/pcl/${SUBSYS_NAME}/poisson/octree.h
    	include/pcl/${SUBSYS_NAME}/poisson/polynomial.h
    	include/pcl/${SUBSYS_NAME}/poisson/ppolynomial.h
    	include/pcl/${SUBSYS_NAME}/poisson/sparse_matrix.h
    	include/pcl/${SUBSYS_NAME}/poisson/time.h
    	include/pcl/${SUBSYS_NAME}/poisson/vector.h
    	)
    set(POISSON_IMPLS
        include/pcl/${SUBSYS_NAME}/impl/poisson/function_data.hpp
        include/pcl/${SUBSYS_NAME}/impl/poisson/geometry.hpp
        include/pcl/${SUBSYS_NAME}/impl/poisson/multi_grid_octree_data.hpp
        include/pcl/${SUBSYS_NAME}/impl/poisson/octree.hpp
        include/pcl/${SUBSYS_NAME}/impl/poisson/polynomial.hpp
        include/pcl/${SUBSYS_NAME}/impl/poisson/ppolynomial.hpp
        include/pcl/${SUBSYS_NAME}/impl/poisson/sparse_matrix.hpp
        include/pcl/${SUBSYS_NAME}/impl/poisson/vector.hpp
    	)
    set(POISSON_SOURCES
    	src/poisson/cmd_line_parser.cpp
    	src/poisson/factor.cpp
    	src/poisson/geometry.cpp
    	src/poisson/marching_cubes.cpp
    	src/poisson/time.cpp
    	)
    
    set(srcs src/processing.cpp 
        src/ear_clipping.cpp
        src/gp3.cpp
        src/grid_projection.cpp
        src/marching_cubes.cpp
        src/marching_cubes_greedy.cpp
        src/marching_cubes_greedy_dot.cpp
        src/mls.cpp
        src/mls_omp.cpp
        src/organized_fast_mesh.cpp
        src/simplification_remove_unused_vertices.cpp
        src/surfel_smoothing.cpp
        src/texture_mapping.cpp
        ${VTK_SMOOTHING_SOURCE}
      #  src/poisson.cpp
        ${HULL_SOURCES}
     #   ${POISSON_SOURCES}
        ${NURBS_SOURCES}
        )

    set(incs include/pcl/${SUBSYS_NAME}/ear_clipping.h
        include/pcl/${SUBSYS_NAME}/gp3.h
        include/pcl/${SUBSYS_NAME}/grid_projection.h
        include/pcl/${SUBSYS_NAME}/marching_cubes.h
        include/pcl/${SUBSYS_NAME}/marching_cubes_greedy.h
        include/pcl/${SUBSYS_NAME}/marching_cubes_greedy_dot.h
        include/pcl/${SUBSYS_NAME}/mls.h
        include/pcl/${SUBSYS_NAME}/mls_omp.h
        include/pcl/${SUBSYS_NAME}/organized_fast_mesh.h
        include/pcl/${SUBSYS_NAME}/reconstruction.h
        include/pcl/${SUBSYS_NAME}/processing.h
        include/pcl/${SUBSYS_NAME}/simplification_remove_unused_vertices.h
        include/pcl/${SUBSYS_NAME}/surfel_smoothing.h        
        include/pcl/${SUBSYS_NAME}/texture_mapping.h
        ${VTK_SMOOTHING_INCLUDES}
      #  include/pcl/${SUBSYS_NAME}/poisson.h
        ${HULL_INCLUDES}
      #  ${POISSON_INCLUDES}
        )

    set(impl_incs include/pcl/${SUBSYS_NAME}/impl/gp3.hpp
        include/pcl/${SUBSYS_NAME}/impl/grid_projection.hpp
        include/pcl/${SUBSYS_NAME}/impl/marching_cubes.hpp
        include/pcl/${SUBSYS_NAME}/impl/marching_cubes_greedy.hpp
        include/pcl/${SUBSYS_NAME}/impl/marching_cubes_greedy_dot.hpp
        include/pcl/${SUBSYS_NAME}/impl/mls.hpp
        include/pcl/${SUBSYS_NAME}/impl/mls_omp.hpp
        include/pcl/${SUBSYS_NAME}/impl/organized_fast_mesh.hpp
        include/pcl/${SUBSYS_NAME}/impl/reconstruction.hpp
        include/pcl/${SUBSYS_NAME}/impl/surfel_smoothing.hpp
        include/pcl/${SUBSYS_NAME}/impl/texture_mapping.hpp
      #  include/pcl/${SUBSYS_NAME}/impl/poisson.hpp
        ${HULL_IMPLS}
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${VTK_INCLUDE_DIRS})
    link_directories(${VTK_LIBRARY_DIRS})
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs} ${NURBS_INCLUDES} ${NURBS_IMPL})
    target_link_libraries(${LIB_NAME} pcl_common pcl_io pcl_search pcl_kdtree pcl_octree ${OpenCV_LIBS} ${VTK_SMOOTHING_TARGET_LINK_LIBRARIES})
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

    if(BUILD_surface_nurbs)
        PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/nurbs ${NURBS_INCLUDES})
    endif(BUILD_surface_nurbs)

endif(build)
