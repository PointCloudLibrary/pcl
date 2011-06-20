set(SUBSYS_NAME search)
set(SUBSYS_DESC "Point cloud generic search library")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(srcs 
        src/generic_search.cpp
        src/kdtree.cpp
        src/organized_neighbor_search.cpp
        )

    set(incs 
        include/pcl/${SUBSYS_NAME}/generic_search.h
        include/pcl/${SUBSYS_NAME}/kdtree.h
        include/pcl/${SUBSYS_NAME}/organized_neighbor_search.h
        )

    set(impl_incs 
        include/pcl/${SUBSYS_NAME}/impl/generic_search.hpp
        include/pcl/${SUBSYS_NAME}/impl/kdtree.hpp
        include/pcl/${SUBSYS_NAME}/impl/organized_neighbor_search.hpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_common ${FLANN_LIBRARIES})
    PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
    list(APPEND EXT_DEPS flann)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
                       "${EXT_DEPS}" "" "" "")

    # Install include files
    set(incs 
        include/pcl/${SUBSYS_NAME}/generic_search.h
        include/pcl/${SUBSYS_NAME}/kdtree.h
        include/pcl/${SUBSYS_NAME}/organized_neighbor_search.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})

    set(impl_incs 
        include/pcl/${SUBSYS_NAME}/impl/generic_search.hpp
        include/pcl/${SUBSYS_NAME}/impl/kdtree.hpp
        include/pcl/${SUBSYS_NAME}/impl/organized_neighbor_search.hpp
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
    
    add_subdirectory(test)
endif(build)

