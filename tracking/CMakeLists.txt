set(SUBSYS_NAME tracking)
set(SUBSYS_DESC "Point cloud tracking library")
set(SUBSYS_DEPS common kdtree)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(srcs 
        src/tracking.cpp
        src/particle_filter.cpp
        )

    set(incs 
        include/pcl/${SUBSYS_NAME}/tracking.h
        include/pcl/${SUBSYS_NAME}/coherence.h
        include/pcl/${SUBSYS_NAME}/nearest_pair_point_cloud_coherence.h
        include/pcl/${SUBSYS_NAME}/distance_coherence.h
        include/pcl/${SUBSYS_NAME}/hsv_color_coherence.h
        include/pcl/${SUBSYS_NAME}/normal_coherence.h
        include/pcl/${SUBSYS_NAME}/particle_filter.h
        )

    set(impl_incs 
        include/pcl/${SUBSYS_NAME}/impl/tracking.hpp
        include/pcl/${SUBSYS_NAME}/impl/coherence.hpp
        include/pcl/${SUBSYS_NAME}/impl/nearest_pair_point_cloud_coherence.hpp
        include/pcl/${SUBSYS_NAME}/impl/distance_coherence.hpp
        include/pcl/${SUBSYS_NAME}/impl/hsv_color_coherence.hpp
        include/pcl/${SUBSYS_NAME}/impl/normal_coherence.hpp
        include/pcl/${SUBSYS_NAME}/impl/particle_filter.hpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    #target_link_libraries(${LIB_NAME})
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "" "" "" "")
    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
endif(build)

