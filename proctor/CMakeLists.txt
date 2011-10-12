set(SUBSYS_NAME proctor)
set(SUBSYS_DESC "Proctor: 3D feature benchmarking")
set(SUBSYS_DEPS common features)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

if(build)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

    set(incs 
        #include/pcl/${SUBSYS_NAME}/foo.h
        )
    
    set(impl_incs 
        #include/pcl/${SUBSYS_NAME}/impl/foo.hpp
        )

    set(srcs 
        src/proctor.cpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    PCL_ADD_OPENMP_FLAGS(${LIB_NAME})
    PCL_LINK_OPENMP(${LIB_NAME})
    target_link_libraries(${LIB_NAME})
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")
    # Install headers
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

    #add_subdirectory(test)
endif(build)
