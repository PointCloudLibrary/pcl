set(SUBSYS_NAME proctor)
set(SUBSYS_DESC "Proctor: 3D feature benchmarking")
set(SUBSYS_DEPS common io filters sample_consensus segmentation search kdtree features surface octree registration visualization keypoints)
set(DEFAULT OFF)
set(REASON "")

if(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
    set(VTK_USE_FILE ${VTK_USE_FILE} CACHE INTERNAL "VTK_USE_FILE")
    include (${VTK_USE_FILE})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
endif(NOT VTK_FOUND)

PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS vtk)

if(build)
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs}) 
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    set(incs 
        include/pcl/${SUBSYS_NAME}/vtkLidarScanner.h
        )
    
    #set(impl_incs 
        ##include/pcl/${SUBSYS_NAME}/impl/foo.hpp
        #)

    #set(srcs 
        #src/proctor.cpp
        #)

    #set(LIB_NAME pcl_${SUBSYS_NAME})
    #PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    #PCL_ADD_SSE_FLAGS(${LIB_NAME})
    #PCL_ADD_OPENMP_FLAGS(${LIB_NAME})
    #PCL_LINK_OPENMP(${LIB_NAME})
    #target_link_libraries(${LIB_NAME})
    #PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")
    ## Install headers
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    #add_subdirectory(test)

    include_directories(${SUBSYS_NAME}/include)
    PCL_ADD_EXECUTABLE (proctor ${SUBSYS_NAME} src/main.cpp src/detector.cpp src/proctor.cpp src/scanner.cpp)
    target_link_libraries(proctor ${FLANN_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_KEYPOINTS_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} vtkCommon vtkFiltering vtkGraphics vtkIO)

endif(build)

