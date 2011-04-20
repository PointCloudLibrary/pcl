set(SUBSYS_NAME octree)
set(SUBSYS_DESC "Point cloud octree library")
set(SUBSYS_DEPS  common )

set(build TRUE)
PCL_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
PCL_SUBSYS_DEPEND(build ${SUBSYS_NAME} ${SUBSYS_DEPS})

if(build)
    set(srcs src/octree_impl.cpp
    )

    set(incs 
        include/pcl/${SUBSYS_NAME}/octree_base.h 
        include/pcl/${SUBSYS_NAME}/octree_impl.h 
        include/pcl/${SUBSYS_NAME}/octree_nodes.h 
        include/pcl/${SUBSYS_NAME}/octree_pointcloud_density.h
        include/pcl/${SUBSYS_NAME}/octree_pointcloud_occupancy.h
        include/pcl/${SUBSYS_NAME}/octree_pointcloud_singlepoint.h
        include/pcl/${SUBSYS_NAME}/octree_pointcloud_pointvector.h
        include/pcl/${SUBSYS_NAME}/octree_pointcloud_changedetector.h
        include/pcl/${SUBSYS_NAME}/octree_pointcloud_voxelcentroid.h
        include/pcl/${SUBSYS_NAME}/octree_pointcloud.h
        include/pcl/${SUBSYS_NAME}/octree.h
        include/pcl/${SUBSYS_NAME}/octree2buf_base.h
        )

    set(impl_incs    
        include/pcl/${SUBSYS_NAME}/impl/octree_base.hpp
        include/pcl/${SUBSYS_NAME}/impl/octree_pointcloud.hpp
        include/pcl/${SUBSYS_NAME}/impl/octree2buf_base.hpp
        )

    set(LIB_NAME pcl_${SUBSYS_NAME})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    PCL_ADD_LIBRARY(${LIB_NAME} ${SUBSYS_NAME} ${srcs} ${incs} ${impl_incs})
    PCL_ADD_SSE_FLAGS(${LIB_NAME})
    target_link_libraries(${LIB_NAME})
    PREFIX_LIST(EXT_DEPS pcl_ "${SUBSYS_DEPS}")
    PCL_MAKE_PKGCONFIG(${LIB_NAME} ${SUBSYS_NAME} "${SUBSYS_DESC}"
        "${EXT_DEPS}" "" "" "")
 
    # Install include files
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME} ${incs})
    PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/impl ${impl_incs})
endif(build)

 
  