set(SUBSYS_NAME search)
set(SUBSYS_DESC "Point cloud generic search library")
set(SUBSYS_DEPS common kdtree octree)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS} EXT_DEPS flann)

if(build)
    set(srcs 
      #src/auto.cpp
        src/kdtree.cpp
        src/organized.cpp
        src/octree.cpp
        )

    set(incs 
        include/pcl/${SUBSYS_NAME}/auto.h
        include/pcl/${SUBSYS_NAME}/search.h
        include/pcl/${SUBSYS_NAME}/kdtree.h
        include/pcl/${SUBSYS_NAME}/organized.h
        include/pcl/${SUBSYS_NAME}/organized_omp.h
        include/pcl/${SUBSYS_NAME}/octree.h
        include/pcl/${SUBSYS_NAME}/pcl_search.h
        include/pcl/${SUBSYS_NAME}/flann_search.h
        )

    set(impl_incs 
        #include/pcl/${SUBSYS_NAME}/impl/auto.hpp
        include/pcl/${SUBSYS_NAME}/impl/flann_search.hpp
        include/pcl/${SUBSYS_NAME}/impl/organized.hpp
        include/pcl/${SUBSYS_NAME}/impl/organized_omp.hpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    PCL_ADD_OPENMP_FLAGS(${LIB_NAME})
    PCL_LINK_OPENMP(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_common ${FLANN_LIBRARIES} pcl_octree pcl_kdtree)
    list(APPEND EXT_DEPS flann)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
    
    add_subdirectory(test)
endif(build)

