set(SUBSYS_NAME reconstruction)
set(SUBSYS_DESC "Point cloud reconstruction library")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC(${SUBSYS_NAME})

if(build)
 
   include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

    set(srcs 
        src/tsdf_volume.cpp
        )

    set(incs 
        include/pcl/${SUBSYS_NAME}/tsdf_volume.h
        )

    set(impl_incs 
        include/pcl/${SUBSYS_NAME}/impl/tsdf_volume.hpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})

    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    PCL_ADD_OPENMP_FLAGS(${LIB_NAME})
    PCL_LINK_OPENMP(${LIB_NAME})

    target_link_libraries(${LIB_NAME} pcl_common)
    
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})

endif(build)


