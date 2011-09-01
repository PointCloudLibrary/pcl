set(SUBSYS_NAME search)
set(SUBSYS_DESC "Point cloud generic search library")
#set(SUBSYS_DEPS common io cuda_common gpu_octree gpu_common octree)
set(SUBSYS_DEPS common octree kdtree)

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(srcs 
        src/auto_tuned_search.cpp
        src/search.cpp
        src/kdtree.cpp
        src/organized_neighbor_search.cpp
        src/octree_impl.cpp
#        src/octree_gpu.cpp
        )

    set(incs 
        include/pcl/${SUBSYS_NAME}/auto_tuned_search.h
        include/pcl/${SUBSYS_NAME}/search.h
        include/pcl/${SUBSYS_NAME}/kdtree.h
        include/pcl/${SUBSYS_NAME}/organized_neighbor_search.h
        include/pcl/${SUBSYS_NAME}/octree_pointcloud.h
 #       include/pcl/${SUBSYS_NAME}/octree_gpu.h
        )

    set(impl_incs 
        include/pcl/${SUBSYS_NAME}/impl/auto_tuned_search.hpp
        include/pcl/${SUBSYS_NAME}/impl/search.hpp
        include/pcl/${SUBSYS_NAME}/impl/kdtree.hpp
        include/pcl/${SUBSYS_NAME}/impl/organized_neighbor_search.hpp
        include/pcl/${SUBSYS_NAME}/impl/octree_pointcloud.hpp
  #      include/pcl/${SUBSYS_NAME}/impl/octree_gpu.hpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME} pcl_common ${FLANN_LIBRARIES} pcl_octree pcl_kdtree)
    list(APPEND EXT_DEPS flann)
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
      "{SUBSYS_DEPS}" "${EXT_DEPS}" "" "" "")

    # Install include files
    set(incs 
        include/pcl/${SUBSYS_NAME}/auto_tuned_search.h
        include/pcl/${SUBSYS_NAME}/search.h
        include/pcl/${SUBSYS_NAME}/kdtree.h
        include/pcl/${SUBSYS_NAME}/organized_neighbor_search.h
        include/pcl/${SUBSYS_NAME}/octree_pointcloud.h
   #     include/pcl/${SUBSYS_NAME}/octree_gpu.h
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})

    set(impl_incs 
        include/pcl/${SUBSYS_NAME}/impl/auto_tuned_search.hpp
        include/pcl/${SUBSYS_NAME}/impl/search.hpp
        include/pcl/${SUBSYS_NAME}/impl/kdtree.hpp
        include/pcl/${SUBSYS_NAME}/impl/organized_neighbor_search.hpp
        include/pcl/${SUBSYS_NAME}/impl/octree_pointcloud.hpp
        #include/pcl/${SUBSYS_NAME}/impl/octree_gpu.hpp
        )
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
    
    add_subdirectory(test)
endif(build)

